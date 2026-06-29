/* Compiler.h — AUTOSAR R23-11 Compiler Abstraction (simplified for GCC/ARM) */
#ifndef COMPILER_H
#define COMPILER_H

/* FUNC(rettype, memclass) — function declaration macro */
#define FUNC(rettype, memclass)   rettype

/* FUNC_P2CONST — pointer to constant data returned by function */
#define FUNC_P2CONST(rettype, ptrclass, memclass) const rettype *

/* VAR — variable declaration */
#define VAR(vartype, memclass)    vartype

/* P2VAR — pointer to variable */
#define P2VAR(ptrtype, memclass, ptrclass) ptrtype *

/* P2CONST — pointer to constant */
#define P2CONST(ptrtype, memclass, ptrclass) const ptrtype *

/* CONSTP2VAR — constant pointer to variable */
#define CONSTP2VAR(ptrtype, memclass, ptrclass) ptrtype * const

/* CONST — constant value */
#define CONST(consttype, memclass) const consttype

/* STATIC */
#define STATIC static

#endif /* COMPILER_H */
