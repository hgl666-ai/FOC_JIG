
# NAS上的Git仓库创建和管理（通过SSH）

## 1、NAS上安装 Git Server 套件，并启用&设置权限 (添加新用户需要来这里设置权限！！！)

## 2、创建 Git 仓库专用的共享文件夹，例如 /volume7/git_repos，并设置权限

## 3、启用NAS的 SSH 服务

## 4、在本地电脑上通过 SSH 连接到 NAS
登录管理员账号后，依次执行下面指令来创建1个仓库：
```bash
# 进入共享文件夹
cd /volume7/git_repos

# 创建仓库目录，仓库名称为 M1_fixture_ZB
mkdir M1_fixture_ZB.git

# 进入仓库目录
cd M1_fixture_ZB.git

# 初始化仓库，类型为裸仓库
git init --bare
```

## 5、在本地电脑建立工程，或找到已有工程
进入工程目录，右键“打开cmd”，或者打开cmd后再cd到工程目录，执行下面指令：
```bash
# 初始化本地仓库
git init

# 若有必要，先创建.gitignore文件，并编辑好

# 添加所有文件到暂存区
git add .
    
# 创建初始提交（这会在本地创建 master 分支）
git commit -m "Initial commit"

# 添加NAS远程仓库地址，这里假设NAS用户为mojun（可以非管理员，只有对共享文件夹有读写权限即可）
git remote add origin ssh://mojun@192.168.31.183:/volume7/git_repos/M1_fixture_ZB.git

# 查看远程仓库地址
git remote -v

# 首次推送，需要设置上游分支（upstream）。这里会被要求输入NAS用户的登录密码。
git push -u origin master
```

## 6、后续推送的步骤
```bash
# 添加所有文件到暂存区
git add .

# 提交
git commit -m "commit message"

# 推送
git push
```

## 7、他人克隆仓库的步骤
```bash
# 使用SSH连接克隆，克隆时需要输入用户密码（请替换用户名mojun）
git clone ssh://mojun@192.168.31.183:/volume7/git_repos/M1_fixture_ZB.git
```

## 8、在旧提交版本上创建新分支
```bash
# 在旧提交版本 78f31e0 上创建新分支 support-v2
git branch support-v2 78f31e0
# 切换到新分支
git checkout support-v2
# 推送新分支到远程仓库
git push origin support-v2

# 在另外一台电脑上，克隆仓库，只克隆该分支，进行维护工作
git clone -b support-v2 --single-branch ssh://mojun@192.168.31.183:/volume7/git_repos/M1_fixture_ZB.git
```

# NAS上的Git仓库如何配置SSH秘钥，使得推送和克隆不再需要输入密码？
未研究出来，按照官方步骤去操作也没有成功，还是需要输入密码


# NAS上的Git仓库配置HTTP/HTTPS链接
DS1621+服务器原生不支持Git仓库的HTTP/HTTPS链接，这里不再折腾
