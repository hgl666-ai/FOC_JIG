
#ifndef _M_MACROS_H_
#define _M_MACROS_H_

// Use NUM_ARGS(__VA_ARGS__) to get the number of variadic arguments
#define _NUM_ARGS(_,n,m,l,k,j,i,h,g,f,e,d,c,b,a,Z,Y,X,W,V,U,T,S,R,Q,P,O,N,M,L,K,J,I,H,G,F,E,D,C,B,A,OUT,...) OUT
#define NUM_ARGS(V...) _NUM_ARGS(0,V,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)

// Macros to chain up to 40 conditions
#define _DO_1(W,C,A)       (_##W##_1(A))
#define _DO_2(W,C,A,B)     (_##W##_1(A) C _##W##_1(B))
#define _DO_3(W,C,A,V...)  (_##W##_1(A) C _DO_2(W,C,V))
#define _DO_4(W,C,A,V...)  (_##W##_1(A) C _DO_3(W,C,V))
#define _DO_5(W,C,A,V...)  (_##W##_1(A) C _DO_4(W,C,V))
#define _DO_6(W,C,A,V...)  (_##W##_1(A) C _DO_5(W,C,V))
#define _DO_7(W,C,A,V...)  (_##W##_1(A) C _DO_6(W,C,V))
#define _DO_8(W,C,A,V...)  (_##W##_1(A) C _DO_7(W,C,V))
#define _DO_9(W,C,A,V...)  (_##W##_1(A) C _DO_8(W,C,V))
#define _DO_10(W,C,A,V...) (_##W##_1(A) C _DO_9(W,C,V))
#define _DO_11(W,C,A,V...) (_##W##_1(A) C _DO_10(W,C,V))
#define _DO_12(W,C,A,V...) (_##W##_1(A) C _DO_11(W,C,V))
#define _DO_13(W,C,A,V...) (_##W##_1(A) C _DO_12(W,C,V))
#define _DO_14(W,C,A,V...) (_##W##_1(A) C _DO_13(W,C,V))
#define _DO_15(W,C,A,V...) (_##W##_1(A) C _DO_14(W,C,V))
#define _DO_16(W,C,A,V...) (_##W##_1(A) C _DO_15(W,C,V))
#define _DO_17(W,C,A,V...) (_##W##_1(A) C _DO_16(W,C,V))
#define _DO_18(W,C,A,V...) (_##W##_1(A) C _DO_17(W,C,V))
#define _DO_19(W,C,A,V...) (_##W##_1(A) C _DO_18(W,C,V))
#define _DO_20(W,C,A,V...) (_##W##_1(A) C _DO_19(W,C,V))
#define _DO_21(W,C,A,V...) (_##W##_1(A) C _DO_20(W,C,V))
#define _DO_22(W,C,A,V...) (_##W##_1(A) C _DO_21(W,C,V))
#define _DO_23(W,C,A,V...) (_##W##_1(A) C _DO_22(W,C,V))
#define _DO_24(W,C,A,V...) (_##W##_1(A) C _DO_23(W,C,V))
#define _DO_25(W,C,A,V...) (_##W##_1(A) C _DO_24(W,C,V))
#define _DO_26(W,C,A,V...) (_##W##_1(A) C _DO_25(W,C,V))
#define _DO_27(W,C,A,V...) (_##W##_1(A) C _DO_26(W,C,V))
#define _DO_28(W,C,A,V...) (_##W##_1(A) C _DO_27(W,C,V))
#define _DO_29(W,C,A,V...) (_##W##_1(A) C _DO_28(W,C,V))
#define _DO_30(W,C,A,V...) (_##W##_1(A) C _DO_29(W,C,V))
#define _DO_31(W,C,A,V...) (_##W##_1(A) C _DO_30(W,C,V))
#define _DO_32(W,C,A,V...) (_##W##_1(A) C _DO_31(W,C,V))
#define _DO_33(W,C,A,V...) (_##W##_1(A) C _DO_32(W,C,V))
#define _DO_34(W,C,A,V...) (_##W##_1(A) C _DO_33(W,C,V))
#define _DO_35(W,C,A,V...) (_##W##_1(A) C _DO_34(W,C,V))
#define _DO_36(W,C,A,V...) (_##W##_1(A) C _DO_35(W,C,V))
#define _DO_37(W,C,A,V...) (_##W##_1(A) C _DO_36(W,C,V))
#define _DO_38(W,C,A,V...) (_##W##_1(A) C _DO_37(W,C,V))
#define _DO_39(W,C,A,V...) (_##W##_1(A) C _DO_38(W,C,V))
#define _DO_40(W,C,A,V...) (_##W##_1(A) C _DO_39(W,C,V))
#define __DO_N(W,C,N,V...) _DO_##N(W,C,V)
#define _DO_N(W,C,N,V...)  __DO_N(W,C,N,V)
#define DO(W,C,V...)       (_DO_N(W,C,NUM_ARGS(V),V))

#if 1
    #define FIRST(a,...)     a
    #define SECOND(a,b,...)  b
    #define THIRD(a,b,c,...) c

    #define IS_PROBE(V...) SECOND(V, 0)     // Get the second item passed, or 0
    #define PROBE() ~, 1                    // Second item will be 1 if this is passed
    #define _NOT_0 PROBE()
    #define NOT(x) IS_PROBE(_CAT(_NOT_, x)) //   NOT('0') gets '1'. Anything else gets '0'.
    #define _BOOL(x) NOT(NOT(x))            // _BOOL('0') gets '0'. Anything else gets '1'.

    // Concatenate symbol names, without or with pre-expansion
    #define _CAT(a,V...) a##V
    #define CAT(a,V...) _CAT(a,V)

    // Recognize "true" values: blank, 1, 0x1, true
    #define _ISENA_     ~,1
    #define _ISENA_1    ~,1
    #define _ISENA_0x1  ~,1
    #define _ISENA_true ~,1
    #define _ISENA(V...)        IS_PROBE(V)

    // Macros to evaluate simple option switches
    #define _ENA_1(O)           _ISENA(CAT(_IS,CAT(ENA_, O)))
    #define _DIS_1(O)           NOT(_ENA_1(O))
    #define ENABLED(V...)       DO(ENA,&&,V)
    #define DISABLED(V...)      DO(DIS,&&,V)
    #define ANY(V...)          !DISABLED(V)
#endif

#define _ULL_MASK_1(O)      (1ULL<<(O))
#define _UL_MASK_1(O)       (1UL<<(O))
#define _U_MASK_1(O)        (1U<<(O))

// FOR 32 BITS SYSTEM
#define MASK64(V...)        DO(ULL_MASK,|,V)
#define MASK32(V...)        DO(UL_MASK,|,V)


// Expansion of some code
#define CODE_16( A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,...) A; B; C; D; E; F; G; H; I; J; K; L; M; N; O; P
#define CODE_15( A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,...) A; B; C; D; E; F; G; H; I; J; K; L; M; N; O
#define CODE_14( A,B,C,D,E,F,G,H,I,J,K,L,M,N,...) A; B; C; D; E; F; G; H; I; J; K; L; M; N
#define CODE_13( A,B,C,D,E,F,G,H,I,J,K,L,M,...) A; B; C; D; E; F; G; H; I; J; K; L; M
#define CODE_12( A,B,C,D,E,F,G,H,I,J,K,L,...) A; B; C; D; E; F; G; H; I; J; K; L
#define CODE_11( A,B,C,D,E,F,G,H,I,J,K,...) A; B; C; D; E; F; G; H; I; J; K
#define CODE_10( A,B,C,D,E,F,G,H,I,J,...) A; B; C; D; E; F; G; H; I; J
#define CODE_9( A,B,C,D,E,F,G,H,I,...) A; B; C; D; E; F; G; H; I
#define CODE_8( A,B,C,D,E,F,G,H,...) A; B; C; D; E; F; G; H
#define CODE_7( A,B,C,D,E,F,G,...) A; B; C; D; E; F; G
#define CODE_6( A,B,C,D,E,F,...) A; B; C; D; E; F
#define CODE_5( A,B,C,D,E,...) A; B; C; D; E
#define CODE_4( A,B,C,D,...) A; B; C; D
#define CODE_3( A,B,C,...) A; B; C
#define CODE_2( A,B,...) A; B
#define CODE_1( A,...) A
#define CODE_0(...)
#define _CODE_N(N,V...) CODE_##N(V)
#define CODE_N(N,V...) _CODE_N(N,V)

// Expansion of some non-delimited content
#define GANG_16(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,...) A B C D E F G H I J K L M N O P
#define GANG_15(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,...) A B C D E F G H I J K L M N O
#define GANG_14(A,B,C,D,E,F,G,H,I,J,K,L,M,N,...) A B C D E F G H I J K L M N
#define GANG_13(A,B,C,D,E,F,G,H,I,J,K,L,M...) A B C D E F G H I J K L M
#define GANG_12(A,B,C,D,E,F,G,H,I,J,K,L...) A B C D E F G H I J K L
#define GANG_11(A,B,C,D,E,F,G,H,I,J,K,...) A B C D E F G H I J K
#define GANG_10(A,B,C,D,E,F,G,H,I,J,...) A B C D E F G H I J
#define GANG_9( A,B,C,D,E,F,G,H,I,...) A B C D E F G H I
#define GANG_8( A,B,C,D,E,F,G,H,...) A B C D E F G H
#define GANG_7( A,B,C,D,E,F,G,...) A B C D E F G
#define GANG_6( A,B,C,D,E,F,...) A B C D E F
#define GANG_5( A,B,C,D,E,...) A B C D E
#define GANG_4( A,B,C,D,...) A B C D
#define GANG_3( A,B,C,...) A B C
#define GANG_2( A,B,...) A B
#define GANG_1( A,...) A
#define GANG_0(...)
#define _GANG_N(N,V...) GANG_##N(V)
#define GANG_N(N,V...) _GANG_N(N,V)
#define GANG_N_1(N,K) _GANG_N(N,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K)

// Expansion of some list items
#define LIST_26(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z
#define LIST_25(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y
#define LIST_24(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X
#define LIST_23(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W
#define LIST_22(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V
#define LIST_21(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U
#define LIST_20(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T
#define LIST_19(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S
#define LIST_18(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R
#define LIST_17(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q
#define LIST_16(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P
#define LIST_15(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N,O
#define LIST_14(A,B,C,D,E,F,G,H,I,J,K,L,M,N,...) A,B,C,D,E,F,G,H,I,J,K,L,M,N
#define LIST_13(A,B,C,D,E,F,G,H,I,J,K,L,M,...) A,B,C,D,E,F,G,H,I,J,K,L,M
#define LIST_12(A,B,C,D,E,F,G,H,I,J,K,L,...) A,B,C,D,E,F,G,H,I,J,K,L
#define LIST_11(A,B,C,D,E,F,G,H,I,J,K,...) A,B,C,D,E,F,G,H,I,J,K
#define LIST_10(A,B,C,D,E,F,G,H,I,J,...) A,B,C,D,E,F,G,H,I,J
#define LIST_9( A,B,C,D,E,F,G,H,I,...) A,B,C,D,E,F,G,H,I
#define LIST_8( A,B,C,D,E,F,G,H,...) A,B,C,D,E,F,G,H
#define LIST_7( A,B,C,D,E,F,G,...) A,B,C,D,E,F,G
#define LIST_6( A,B,C,D,E,F,...) A,B,C,D,E,F
#define LIST_5( A,B,C,D,E,...) A,B,C,D,E
#define LIST_4( A,B,C,D,...) A,B,C,D
#define LIST_3( A,B,C,...) A,B,C
#define LIST_2( A,B,...) A,B
#define LIST_1( A,...) A
#define LIST_0(...)

#define _LIST_N(N,V...) LIST_##N(V)
#define LIST_N(N,V...) _LIST_N(N,V)
#define LIST_N_1(N,K) _LIST_N(N,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K,K)
#define ARRAY_N(N,V...) { _LIST_N(N,V) }
#define ARRAY_N_1(N,K)  { LIST_N_1(N,K) }

#define LIST_SUFFIX_20(SU,A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,...) \
            A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU,K##SU,\
            L##SU,M##SU,N##SU,O##SU,P##SU,Q##SU,R##SU,S##SU,T##SU
#define LIST_SUFFIX_19(SU,A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,...) \
            A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU,K##SU,\
            L##SU,M##SU,N##SU,O##SU,P##SU,Q##SU,R##SU,S##SU
#define LIST_SUFFIX_18(SU,A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,...) \
            A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU,K##SU,\
            L##SU,M##SU,N##SU,O##SU,P##SU,Q##SU,R##SU
#define LIST_SUFFIX_17(SU,A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,...) \
            A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU,K##SU,\
            L##SU,M##SU,N##SU,O##SU,P##SU,Q##SU
#define LIST_SUFFIX_16(SU,A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,...) \
            A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU,K##SU,\
            L##SU,M##SU,N##SU,O##SU,P##SU
#define LIST_SUFFIX_15(SU,A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,...) \
            A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU,K##SU,\
            L##SU,M##SU,N##SU,O##SU
#define LIST_SUFFIX_14(SU,A,B,C,D,E,F,G,H,I,J,K,L,M,N,...) A##SU,B##SU,C##SU,\
            D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU,K##SU,L##SU,M##SU,N##SU
#define LIST_SUFFIX_13(SU,A,B,C,D,E,F,G,H,I,J,K,L,M,...) A##SU,B##SU,C##SU,\
            D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU,K##SU,L##SU,M##SU
#define LIST_SUFFIX_12(SU,A,B,C,D,E,F,G,H,I,J,K,L,...) A##SU,B##SU,C##SU,\
            D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU,K##SU,L##SU
#define LIST_SUFFIX_11(SU,A,B,C,D,E,F,G,H,I,J,K,...) A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU,K##SU
#define LIST_SUFFIX_10(SU,A,B,C,D,E,F,G,H,I,J,...) A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU,H##SU,I##SU,J##SU
#define LIST_SUFFIX_9( SU,A,B,C,D,E,F,G,H,I,...) A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU,H##SU,I##SU
#define LIST_SUFFIX_8( SU,A,B,C,D,E,F,G,H,...) A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU,H##SU
#define LIST_SUFFIX_7( SU,A,B,C,D,E,F,G,...) A##SU,B##SU,C##SU,D##SU,E##SU,F##SU,G##SU
#define LIST_SUFFIX_6( SU,A,B,C,D,E,F,...) A##SU,B##SU,C##SU,D##SU,E##SU,F##SU
#define LIST_SUFFIX_5( SU,A,B,C,D,E,...) A##SU,B##SU,C##SU,D##SU,E##SU
#define LIST_SUFFIX_4( SU,A,B,C,D,...) A##SU,B##SU,C##SU,D##SU
#define LIST_SUFFIX_3( SU,A,B,C,...) A##SU,B##SU,C##SU
#define LIST_SUFFIX_2( SU,A,B,...) A##SU,B##SU
#define LIST_SUFFIX_1( SU,A,...) A##SU
#define LIST_SUFFIX_0(...)

#define LIST_SUFFIX_N(N,SU,V...) LIST_SUFFIX_##N(SU,V)
#define ARRAY_SUFFIX_N(N,SU,V...) { LIST_SUFFIX_N(N,SU,V) }

#define LIST_I_0(...)
#define LIST_I_1(A)  A##0
#define LIST_I_2(A)  LIST_I_1(A),A##1
#define LIST_I_3(A)  LIST_I_2(A),A##2
#define LIST_I_4(A)  LIST_I_3(A),A##3
#define LIST_I_5(A)  LIST_I_4(A),A##4
#define LIST_I_6(A)  LIST_I_5(A),A##5
#define LIST_I_7(A)  LIST_I_6(A),A##6
#define LIST_I_8(A)  LIST_I_7(A),A##7
#define LIST_I_9(A)  LIST_I_8(A),A##8
#define LIST_I_10(A) LIST_I_9(A),A##9
#define LIST_I_11(A) LIST_I_10(A),A##10
#define LIST_I_12(A) LIST_I_11(A),A##11
#define LIST_I_13(A) LIST_I_12(A),A##12
#define LIST_I_14(A) LIST_I_13(A),A##13
#define LIST_I_15(A) LIST_I_14(A),A##14
#define LIST_I_16(A) LIST_I_15(A),A##15
#define LIST_I_17(A) LIST_I_16(A),A##16
#define LIST_I_18(A) LIST_I_17(A),A##17
#define LIST_I_19(A) LIST_I_18(A),A##18
#define LIST_I_20(A) LIST_I_19(A),A##19
#define LIST_I_21(A) LIST_I_20(A),A##20
#define LIST_I_22(A) LIST_I_21(A),A##21
#define LIST_I_23(A) LIST_I_22(A),A##22
#define LIST_I_24(A) LIST_I_23(A),A##23
#define LIST_I_25(A) LIST_I_24(A),A##24
#define LIST_I_26(A) LIST_I_25(A),A##25

#define LIST_I_N(N,A)   LIST_I_##N(A)
#define ARRAY_I_N(N,A)  { LIST_I_N(N,A) }

#define math_abs(x)        ((x)>0?(x):-(x))
#define math_max(x, y)     ((x)>(y)?(x):(y))
#define math_min(x, y)     ((x)<(y)?(x):(y))
#define math_limit(x, mi, ma)    ((x)<(mi)?(mi):((x)>(ma)?(ma):(x)))
#define math_abs_subtract(x, y)   ((x)>(y)?((x)-(y)):((y)-(x)))

#define BIT_EN(d, bit)     (!!((d) & (1 << bit)))

#endif // _M_MACROS_H_
