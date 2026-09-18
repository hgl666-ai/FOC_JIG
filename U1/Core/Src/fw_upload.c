/**
  ******************************************************************************
  * @file    fw_upload.c
  * @brief   固件上传三段式状态机 — 双槽 (Slot0=Bootloader, Slot1=App)
  *
  * 边缘处理 (与协议状态码一一对应):
  *   BUSY       : 非 IDLE 收到 START            → 0x01
  *   ERR_SIZE   : LEN<9 / size=0 / size 越界 / 帧数0 → 0x02
  *   ERR_FLASH  : 擦除或写入返回失败             → 0x03, 锁存并中止
  *   ERR_OVERFLOW: 累计字节数超过声明大小         → 0x0D, 锁存
  *   ERR_COUNT  : END 时实际字节数 ≠ 声明字节数   → 0x0A, 不写完整标志
  *   ERR_TIMEOUT: 10s 无新数据                   → 0x0C
  * 锁存语义: 一旦 DATA 帧出错, 后续 DATA 帧一律回原错误码且不写 Flash,
  *          直到新一轮 START 或复位。
  ******************************************************************************
  */
#include "fw_upload.h"
#include "app.h"
#include "flash_map.h"
#include "gd25q128.h"
#include "main.h"
#include <string.h>

/*===== 上传会话状态 =====*/
static uint32_t upload_addr         = 0u;   /* 当前 Flash 写入地址 (base+8 起) */
static uint32_t upload_total        = 0u;   /* 声明的文件大小 (字节)            */
static uint32_t upload_count        = 0u;   /* 已写入字节数                    */
static uint8_t  upload_version[3]   = {0};
static uint16_t page_idx            = 0u;
static uint8_t  upload_slot         = 0u;   /* 0=Bootloader, 1=App             */
static uint32_t upload_last_tick    = 0u;
static uint8_t  upload_error        = 0u;   /* 0=正常, 非0=错误码 (锁存)        */

static uint8_t  page_buf[APP_PAGE_SIZE];

uint8_t FwUpload_CurrentSlot(void)
{
    return upload_slot;
}

/*===== 攒页写入: 满 256B 才提交一次页编程 =====*/
static uint8_t UploadData(const uint8_t *data, uint16_t len)
{
    if (upload_count + (uint32_t)len > upload_total)
        return APP_STATUS_ERR_OVERFLOW;

    for (uint16_t i = 0; i < len; i++) {
        page_buf[page_idx++] = data[i];
        upload_count++;
        if (page_idx >= APP_PAGE_SIZE) {
            if (!GD25Q_WritePage(page_buf, upload_addr, APP_PAGE_SIZE))
                return APP_STATUS_ERR_FLASH;
            upload_addr += APP_PAGE_SIZE;
            page_idx = 0u;
        }
    }
    return APP_STATUS_OK;
}

void FwUpload_Init(void)
{
    upload_slot         = 0u;
    upload_addr         = 0u;
    upload_total        = 0u;
    upload_count        = 0u;
    upload_error        = 0u;
    upload_last_tick    = 0u;
    page_idx            = 0u;
    memset(page_buf, 0, sizeof(page_buf));
}

uint8_t FwUpload_Handle(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    switch (cmd) {

    /*────────────────── 起始帧 (LEN=9) ──────────────────
     * [0..1] u16 总帧数 (大端)
     * [2..5] u32 文件大小 (大端)
     * [6..8] 3B  版本号 [主].[次].[修订]
     */
    case MGR_CMD_FW_START: {
        if (app_state != APP_STATE_IDLE) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(cmd, &e, 1); break;
        }
        if (len < 9u) {
            uint8_t e = APP_STATUS_ERR_SIZE; APP_SendAck(cmd, &e, 1); break;
        }

        uint16_t frames = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
        uint32_t bytes  = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16)
                        | ((uint32_t)data[4] << 8)  |  (uint32_t)data[5];

        if (bytes == 0u || bytes > FW_MAX_IMAGE_SIZE || frames == 0u) {
            uint8_t e = APP_STATUS_ERR_SIZE; APP_SendAck(cmd, &e, 1); break;
        }

        /* SPI Flash 在线自检 (读 JEDEC ID 校验制造商与容量)。
         * 放在这里而不是上电时做: 让"Flash 不在线/型号不对"在**第一条命令**
         * 就以 ERR_FLASH 暴露出来, 而不是等到擦除失败、或上传到一半才报错。
         * 注: 只校验 ID, 不做读写往返, 单次开销约几十微秒。 */
        if (!GD25Q_Test()) {
            uint8_t e = APP_STATUS_ERR_FLASH; APP_SendAck(cmd, &e, 1); break;
        }

        uint32_t base = (upload_slot == 0u) ? FW_SLOT0_BASE : FW_SLOT1_BASE;
        upload_addr         = base + FW_HEAD_SIZE;
        upload_total        = bytes;
        upload_count        = 0u;
        upload_version[0]   = data[6];
        upload_version[1]   = data[7];
        upload_version[2]   = data[8];
        page_idx            = 0u;
        upload_error        = 0u;
        upload_last_tick    = HAL_GetTick();
        memset(page_buf, 0, sizeof(page_buf));   /* 清残页, 防跨会话污染 */

        /* 解除块保护: BP 位保护会导致擦除/写入静默失败 */
        GD25Q_Unprotect();

        /* 擦除覆盖 头部 + 数据 的扇区 */
        {
            uint32_t sec_start = base / 4096u;
            uint32_t sec_count = (bytes + FW_HEAD_SIZE + 4095u) / 4096u;
            for (uint32_t s = sec_start; s < sec_start + sec_count; s++) {
                if (!GD25Q_EraseSector(s * 4096u)) {
                    uint8_t e = APP_STATUS_ERR_FLASH; APP_SendAck(cmd, &e, 1);
                    return 1u;
                }
            }
        }

        /* 写头部 7B: [4B 大小 小端][3B 版本号], 完整标志留到 END 再写 */
        {
            uint8_t hdr[FW_HEAD_SIZE];
            hdr[0] = (uint8_t)(bytes);
            hdr[1] = (uint8_t)(bytes >> 8);
            hdr[2] = (uint8_t)(bytes >> 16);
            hdr[3] = (uint8_t)(bytes >> 24);
            hdr[4] = upload_version[0];
            hdr[5] = upload_version[1];
            hdr[6] = upload_version[2];
            if (!GD25Q_WritePage(hdr, base, FW_HEAD_SIZE - 1u)) {
                uint8_t e = APP_STATUS_ERR_FLASH; APP_SendAck(cmd, &e, 1); break;
            }
        }

        app_state = APP_STATE_UPLOADING;
        { uint8_t ok = APP_STATUS_OK; APP_SendAck(cmd, &ok, 1); }
        break;
    }

    /*────────────────── 数据帧 ──────────────────*/
    case MGR_CMD_FW_DATA: {
        if (app_state != APP_STATE_UPLOADING) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(cmd, &e, 1); break;
        }
        if (upload_error != 0u) {
            /* 错误已锁存: 拒绝继续写入, 只回原错误码 */
            uint8_t e = upload_error; APP_SendAck(cmd, &e, 1); break;
        }

        upload_last_tick = HAL_GetTick();

        {
            uint8_t r = UploadData(data, len);
            if (r != APP_STATUS_OK) upload_error = r;
        }
        {
            uint8_t resp = (upload_error == 0u) ? APP_STATUS_OK : upload_error;
            APP_SendAck(cmd, &resp, 1);
        }
        break;
    }

    /*────────────────── 结束帧 ──────────────────
     * 收尾: 1) 刷残页  2) 比对字节数  3) 写完整标志  4) 切槽
     */
    case MGR_CMD_FW_END: {
        if (app_state != APP_STATE_UPLOADING) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(cmd, &e, 1); break;
        }

        uint8_t  resp = APP_STATUS_OK;
        uint8_t  cur_slot = upload_slot;

        if (upload_error != 0u) {
            resp = upload_error;
        } else if (page_idx > 0u) {
            if (!GD25Q_WritePage(page_buf, upload_addr, page_idx))
                resp = APP_STATUS_ERR_FLASH;
        }

        /* 比对声明字节数 — 只有前面都成功时才判定, 否则保留原错误码 */
        if (resp == APP_STATUS_OK && upload_count != upload_total)
            resp = APP_STATUS_ERR_COUNT;

        /* 完整标志只在字节数匹配时写入 (不变量 I2) */
        if (resp == APP_STATUS_OK) {
            uint8_t hdr[FW_HEAD_SIZE];
            Flash_WriteCompleteFlag(cur_slot, FW_FLAG_COMPLETE);
            /* 回读校验: 完整标志必须真的变成 0xA5 */
            GD25Q_ReadBuffer(hdr, (cur_slot == 0u) ? FW_SLOT0_BASE : FW_SLOT1_BASE, FW_HEAD_SIZE);
            if (hdr[FW_HEAD_OFF_FLAG] != FW_FLAG_COMPLETE)
                resp = APP_STATUS_ERR_FLASH;
        }

        /* 切槽, 为第二次上传 (App) 做准备 */
        upload_slot = (upload_slot == 0u) ? 1u : 0u;

        app_state = APP_STATE_IDLE;
        APP_SendAck(cmd, &resp, 1);
        break;
    }

    default:
        return 0u;      /* 非上传命令, 交给上层继续分发 */
    }

    return 1u;          /* 本命令已被上传模块处理 */
}

void FwUpload_TimeoutCheck(void)
{
    if (app_state != APP_STATE_UPLOADING) return;

    if ((HAL_GetTick() - upload_last_tick) > APP_UPLOAD_TIMEOUT_MS) {
        uint8_t e = (upload_error != 0u) ? upload_error : APP_STATUS_ERR_TIMEOUT;

        if (upload_error == 0u) upload_error = APP_STATUS_ERR_TIMEOUT;
        app_state = APP_STATE_IDLE;

        /* 错误码经 ACK 回传 (SEQ 沿用最后一帧序号) */
        APP_SendAck(MGR_CMD_FW_DATA, &e, 1);
    }
}
