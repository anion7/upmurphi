/* A Bison parser, made by GNU Bison 3.8.2.  */

/* Bison implementation for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015, 2018-2021 Free Software Foundation,
   Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* DO NOT RELY ON FEATURES THAT ARE NOT DOCUMENTED in the manual,
   especially those whose name start with YY_ or yy_.  They are
   private implementation details that can be changed or removed.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output, and Bison version.  */
#define YYBISON 30802

/* Bison version string.  */
#define YYBISON_VERSION "3.8.2"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1




/* First part of user prologue.  */
#line 17 "pddl+.yacc"

/*
Error reporting:
Intention is to provide error token on most bracket expressions,
so synchronisation can occur on next CLOSE_BRAC.
Hence error should be generated for innermost expression containing error.
Expressions which cause errors return a NULL values, and parser
always attempts to carry on.
This won't behave so well if CLOSE_BRAC is missing.

Naming conventions:
Generally, the names should be similar to the PDDL2.1 spec.
During development, they have also been based on older PDDL specs,
older PDDL+ and TIM parsers, and this shows in places.

All the names of fields in the semantic value type begin with t_
Corresponding categories in the grammar begin with c_
Corresponding classes have no prefix.

PDDL grammar       yacc grammar      type of corresponding semantic val.  

thing+             c_things          thing_list
(thing+)           c_thing_list      thing_list

*/

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <ctype.h>

// This is now copied locally to avoid relying on installation 
// of flex++.

//#include "FlexLexer.h"
//#include <FlexLexer.h>

#include "ptree.h"
#include "parse_error.h"

#define YYDEBUG 1 

int yyerror(char *);


extern int yylex();

using namespace PDDL2UPMurphi_parser;


#line 122 "pddl+.cpp"

# ifndef YY_CAST
#  ifdef __cplusplus
#   define YY_CAST(Type, Val) static_cast<Type> (Val)
#   define YY_REINTERPRET_CAST(Type, Val) reinterpret_cast<Type> (Val)
#  else
#   define YY_CAST(Type, Val) ((Type) (Val))
#   define YY_REINTERPRET_CAST(Type, Val) ((Type) (Val))
#  endif
# endif
# ifndef YY_NULLPTR
#  if defined __cplusplus
#   if 201103L <= __cplusplus
#    define YY_NULLPTR nullptr
#   else
#    define YY_NULLPTR 0
#   endif
#  else
#   define YY_NULLPTR ((void*)0)
#  endif
# endif


/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int yydebug;
#endif

/* Token kinds.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    YYEMPTY = -2,
    YYEOF = 0,                     /* "end of file"  */
    YYerror = 256,                 /* error  */
    YYUNDEF = 257,                 /* "invalid token"  */
    OPEN_BRAC = 258,               /* OPEN_BRAC  */
    CLOSE_BRAC = 259,              /* CLOSE_BRAC  */
    OPEN_SQ = 260,                 /* OPEN_SQ  */
    CLOSE_SQ = 261,                /* CLOSE_SQ  */
    DEFINE = 262,                  /* DEFINE  */
    PDDLDOMAIN = 263,              /* PDDLDOMAIN  */
    REQS = 264,                    /* REQS  */
    EQUALITY = 265,                /* EQUALITY  */
    STRIPS = 266,                  /* STRIPS  */
    ADL = 267,                     /* ADL  */
    NEGATIVE_PRECONDITIONS = 268,  /* NEGATIVE_PRECONDITIONS  */
    TYPING = 269,                  /* TYPING  */
    DISJUNCTIVE_PRECONDS = 270,    /* DISJUNCTIVE_PRECONDS  */
    EXT_PRECS = 271,               /* EXT_PRECS  */
    UNIV_PRECS = 272,              /* UNIV_PRECS  */
    QUANT_PRECS = 273,             /* QUANT_PRECS  */
    COND_EFFS = 274,               /* COND_EFFS  */
    FLUENTS = 275,                 /* FLUENTS  */
    TIME = 276,                    /* TIME  */
    DURATIVE_ACTIONS = 277,        /* DURATIVE_ACTIONS  */
    DURATION_INEQUALITIES = 278,   /* DURATION_INEQUALITIES  */
    CONTINUOUS_EFFECTS = 279,      /* CONTINUOUS_EFFECTS  */
    DERIVED_PREDICATES = 280,      /* DERIVED_PREDICATES  */
    TIMED_INITIAL_LITERALS = 281,  /* TIMED_INITIAL_LITERALS  */
    PREFERENCES = 282,             /* PREFERENCES  */
    CONSTRAINTS = 283,             /* CONSTRAINTS  */
    ACTION = 284,                  /* ACTION  */
    PROCESS = 285,                 /* PROCESS  */
    EVENT = 286,                   /* EVENT  */
    DURATIVE_ACTION = 287,         /* DURATIVE_ACTION  */
    DERIVED = 288,                 /* DERIVED  */
    CONSTANTS = 289,               /* CONSTANTS  */
    PREDS = 290,                   /* PREDS  */
    FUNCTIONS = 291,               /* FUNCTIONS  */
    TYPES = 292,                   /* TYPES  */
    ARGS = 293,                    /* ARGS  */
    PRE = 294,                     /* PRE  */
    CONDITION = 295,               /* CONDITION  */
    PREFERENCE = 296,              /* PREFERENCE  */
    START_PRE = 297,               /* START_PRE  */
    END_PRE = 298,                 /* END_PRE  */
    EFFECTS = 299,                 /* EFFECTS  */
    INITIAL_EFFECT = 300,          /* INITIAL_EFFECT  */
    FINAL_EFFECT = 301,            /* FINAL_EFFECT  */
    INVARIANT = 302,               /* INVARIANT  */
    DURATION = 303,                /* DURATION  */
    AT_START = 304,                /* AT_START  */
    AT_END = 305,                  /* AT_END  */
    OVER_ALL = 306,                /* OVER_ALL  */
    AND = 307,                     /* AND  */
    OR = 308,                      /* OR  */
    EXISTS = 309,                  /* EXISTS  */
    FORALL = 310,                  /* FORALL  */
    IMPLY = 311,                   /* IMPLY  */
    NOT = 312,                     /* NOT  */
    WHEN = 313,                    /* WHEN  */
    EITHER = 314,                  /* EITHER  */
    PROBLEM = 315,                 /* PROBLEM  */
    FORDOMAIN = 316,               /* FORDOMAIN  */
    INITIALLY = 317,               /* INITIALLY  */
    OBJECTS = 318,                 /* OBJECTS  */
    GOALS = 319,                   /* GOALS  */
    EQ = 320,                      /* EQ  */
    LENGTH = 321,                  /* LENGTH  */
    SERIAL = 322,                  /* SERIAL  */
    PARALLEL = 323,                /* PARALLEL  */
    METRIC = 324,                  /* METRIC  */
    MINIMIZE = 325,                /* MINIMIZE  */
    MAXIMIZE = 326,                /* MAXIMIZE  */
    HASHT = 327,                   /* HASHT  */
    DURATION_VAR = 328,            /* DURATION_VAR  */
    TOTAL_TIME = 329,              /* TOTAL_TIME  */
    INCREASE = 330,                /* INCREASE  */
    DECREASE = 331,                /* DECREASE  */
    SCALE_UP = 332,                /* SCALE_UP  */
    SCALE_DOWN = 333,              /* SCALE_DOWN  */
    ASSIGN = 334,                  /* ASSIGN  */
    GREATER = 335,                 /* GREATER  */
    GREATEQ = 336,                 /* GREATEQ  */
    LESS = 337,                    /* LESS  */
    LESSEQ = 338,                  /* LESSEQ  */
    Q = 339,                       /* Q  */
    COLON = 340,                   /* COLON  */
    ALWAYS = 341,                  /* ALWAYS  */
    SOMETIME = 342,                /* SOMETIME  */
    WITHIN = 343,                  /* WITHIN  */
    ATMOSTONCE = 344,              /* ATMOSTONCE  */
    SOMETIMEAFTER = 345,           /* SOMETIMEAFTER  */
    SOMETIMEBEFORE = 346,          /* SOMETIMEBEFORE  */
    ALWAYSWITHIN = 347,            /* ALWAYSWITHIN  */
    HOLDDURING = 348,              /* HOLDDURING  */
    HOLDAFTER = 349,               /* HOLDAFTER  */
    ISVIOLATED = 350,              /* ISVIOLATED  */
    BOGUS = 351,                   /* BOGUS  */
    NAME = 352,                    /* NAME  */
    FUNCTION_SYMBOL = 353,         /* FUNCTION_SYMBOL  */
    INTVAL = 354,                  /* INTVAL  */
    FLOATVAL = 355,                /* FLOATVAL  */
    AT_TIME = 356,                 /* AT_TIME  */
    HYPHEN = 357,                  /* HYPHEN  */
    PLUS = 358,                    /* PLUS  */
    MUL = 359,                     /* MUL  */
    DIV = 360,                     /* DIV  */
    UMINUS = 361                   /* UMINUS  */
  };
  typedef enum yytokentype yytoken_kind_t;
#endif

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
union YYSTYPE
{
#line 68 "pddl+.yacc"

    parse_category* t_parse_category;

    effect_lists* t_effect_lists;
    effect* t_effect;
    simple_effect* t_simple_effect;
    cond_effect*   t_cond_effect;
    forall_effect* t_forall_effect;
    timed_effect* t_timed_effect;

    quantifier t_quantifier;
    metric_spec*  t_metric;
    optimization t_optimization;

    symbol* t_symbol;
    var_symbol*   t_var_symbol;
    pddl_type*    t_type;
    pred_symbol*  t_pred_symbol;
    func_symbol*  t_func_symbol;
    const_symbol* t_const_symbol;

    parameter_symbol_list* t_parameter_symbol_list;
    var_symbol_list* t_var_symbol_list;
    const_symbol_list* t_const_symbol_list;
    pddl_type_list* t_type_list;

    proposition* t_proposition;
    pred_decl* t_pred_decl;
    pred_decl_list* t_pred_decl_list;
    func_decl* t_func_decl;
    func_decl_list* t_func_decl_list;

    goal* t_goal;
    con_goal * t_con_goal;
    goal_list* t_goal_list;

    func_term* t_func_term;
    assignment* t_assignment;
    expression* t_expression;
    num_expression* t_num_expression;
    assign_op t_assign_op;
    comparison_op t_comparison_op;

    structure_def* t_structure_def;
    structure_store* t_structure_store;

    action* t_action_def;
    event* t_event_def;
    process* t_process_def;
    durative_action* t_durative_action_def;
    derivation_rule* t_derivation_rule;

    problem* t_problem;
    length_spec* t_length_spec;

    domain* t_domain;    

    pddl_req_flag t_pddl_req_flag;

    plan* t_plan;
    plan_step* t_step;

    int ival;
    double fval;

    char* cp;
    int t_dummy;

    var_symbol_table * vtab;

#line 346 "pddl+.cpp"

};
typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE yylval;


int yyparse (void);



/* Symbol kind.  */
enum yysymbol_kind_t
{
  YYSYMBOL_YYEMPTY = -2,
  YYSYMBOL_YYEOF = 0,                      /* "end of file"  */
  YYSYMBOL_YYerror = 1,                    /* error  */
  YYSYMBOL_YYUNDEF = 2,                    /* "invalid token"  */
  YYSYMBOL_OPEN_BRAC = 3,                  /* OPEN_BRAC  */
  YYSYMBOL_CLOSE_BRAC = 4,                 /* CLOSE_BRAC  */
  YYSYMBOL_OPEN_SQ = 5,                    /* OPEN_SQ  */
  YYSYMBOL_CLOSE_SQ = 6,                   /* CLOSE_SQ  */
  YYSYMBOL_DEFINE = 7,                     /* DEFINE  */
  YYSYMBOL_PDDLDOMAIN = 8,                 /* PDDLDOMAIN  */
  YYSYMBOL_REQS = 9,                       /* REQS  */
  YYSYMBOL_EQUALITY = 10,                  /* EQUALITY  */
  YYSYMBOL_STRIPS = 11,                    /* STRIPS  */
  YYSYMBOL_ADL = 12,                       /* ADL  */
  YYSYMBOL_NEGATIVE_PRECONDITIONS = 13,    /* NEGATIVE_PRECONDITIONS  */
  YYSYMBOL_TYPING = 14,                    /* TYPING  */
  YYSYMBOL_DISJUNCTIVE_PRECONDS = 15,      /* DISJUNCTIVE_PRECONDS  */
  YYSYMBOL_EXT_PRECS = 16,                 /* EXT_PRECS  */
  YYSYMBOL_UNIV_PRECS = 17,                /* UNIV_PRECS  */
  YYSYMBOL_QUANT_PRECS = 18,               /* QUANT_PRECS  */
  YYSYMBOL_COND_EFFS = 19,                 /* COND_EFFS  */
  YYSYMBOL_FLUENTS = 20,                   /* FLUENTS  */
  YYSYMBOL_TIME = 21,                      /* TIME  */
  YYSYMBOL_DURATIVE_ACTIONS = 22,          /* DURATIVE_ACTIONS  */
  YYSYMBOL_DURATION_INEQUALITIES = 23,     /* DURATION_INEQUALITIES  */
  YYSYMBOL_CONTINUOUS_EFFECTS = 24,        /* CONTINUOUS_EFFECTS  */
  YYSYMBOL_DERIVED_PREDICATES = 25,        /* DERIVED_PREDICATES  */
  YYSYMBOL_TIMED_INITIAL_LITERALS = 26,    /* TIMED_INITIAL_LITERALS  */
  YYSYMBOL_PREFERENCES = 27,               /* PREFERENCES  */
  YYSYMBOL_CONSTRAINTS = 28,               /* CONSTRAINTS  */
  YYSYMBOL_ACTION = 29,                    /* ACTION  */
  YYSYMBOL_PROCESS = 30,                   /* PROCESS  */
  YYSYMBOL_EVENT = 31,                     /* EVENT  */
  YYSYMBOL_DURATIVE_ACTION = 32,           /* DURATIVE_ACTION  */
  YYSYMBOL_DERIVED = 33,                   /* DERIVED  */
  YYSYMBOL_CONSTANTS = 34,                 /* CONSTANTS  */
  YYSYMBOL_PREDS = 35,                     /* PREDS  */
  YYSYMBOL_FUNCTIONS = 36,                 /* FUNCTIONS  */
  YYSYMBOL_TYPES = 37,                     /* TYPES  */
  YYSYMBOL_ARGS = 38,                      /* ARGS  */
  YYSYMBOL_PRE = 39,                       /* PRE  */
  YYSYMBOL_CONDITION = 40,                 /* CONDITION  */
  YYSYMBOL_PREFERENCE = 41,                /* PREFERENCE  */
  YYSYMBOL_START_PRE = 42,                 /* START_PRE  */
  YYSYMBOL_END_PRE = 43,                   /* END_PRE  */
  YYSYMBOL_EFFECTS = 44,                   /* EFFECTS  */
  YYSYMBOL_INITIAL_EFFECT = 45,            /* INITIAL_EFFECT  */
  YYSYMBOL_FINAL_EFFECT = 46,              /* FINAL_EFFECT  */
  YYSYMBOL_INVARIANT = 47,                 /* INVARIANT  */
  YYSYMBOL_DURATION = 48,                  /* DURATION  */
  YYSYMBOL_AT_START = 49,                  /* AT_START  */
  YYSYMBOL_AT_END = 50,                    /* AT_END  */
  YYSYMBOL_OVER_ALL = 51,                  /* OVER_ALL  */
  YYSYMBOL_AND = 52,                       /* AND  */
  YYSYMBOL_OR = 53,                        /* OR  */
  YYSYMBOL_EXISTS = 54,                    /* EXISTS  */
  YYSYMBOL_FORALL = 55,                    /* FORALL  */
  YYSYMBOL_IMPLY = 56,                     /* IMPLY  */
  YYSYMBOL_NOT = 57,                       /* NOT  */
  YYSYMBOL_WHEN = 58,                      /* WHEN  */
  YYSYMBOL_EITHER = 59,                    /* EITHER  */
  YYSYMBOL_PROBLEM = 60,                   /* PROBLEM  */
  YYSYMBOL_FORDOMAIN = 61,                 /* FORDOMAIN  */
  YYSYMBOL_INITIALLY = 62,                 /* INITIALLY  */
  YYSYMBOL_OBJECTS = 63,                   /* OBJECTS  */
  YYSYMBOL_GOALS = 64,                     /* GOALS  */
  YYSYMBOL_EQ = 65,                        /* EQ  */
  YYSYMBOL_LENGTH = 66,                    /* LENGTH  */
  YYSYMBOL_SERIAL = 67,                    /* SERIAL  */
  YYSYMBOL_PARALLEL = 68,                  /* PARALLEL  */
  YYSYMBOL_METRIC = 69,                    /* METRIC  */
  YYSYMBOL_MINIMIZE = 70,                  /* MINIMIZE  */
  YYSYMBOL_MAXIMIZE = 71,                  /* MAXIMIZE  */
  YYSYMBOL_HASHT = 72,                     /* HASHT  */
  YYSYMBOL_DURATION_VAR = 73,              /* DURATION_VAR  */
  YYSYMBOL_TOTAL_TIME = 74,                /* TOTAL_TIME  */
  YYSYMBOL_INCREASE = 75,                  /* INCREASE  */
  YYSYMBOL_DECREASE = 76,                  /* DECREASE  */
  YYSYMBOL_SCALE_UP = 77,                  /* SCALE_UP  */
  YYSYMBOL_SCALE_DOWN = 78,                /* SCALE_DOWN  */
  YYSYMBOL_ASSIGN = 79,                    /* ASSIGN  */
  YYSYMBOL_GREATER = 80,                   /* GREATER  */
  YYSYMBOL_GREATEQ = 81,                   /* GREATEQ  */
  YYSYMBOL_LESS = 82,                      /* LESS  */
  YYSYMBOL_LESSEQ = 83,                    /* LESSEQ  */
  YYSYMBOL_Q = 84,                         /* Q  */
  YYSYMBOL_COLON = 85,                     /* COLON  */
  YYSYMBOL_ALWAYS = 86,                    /* ALWAYS  */
  YYSYMBOL_SOMETIME = 87,                  /* SOMETIME  */
  YYSYMBOL_WITHIN = 88,                    /* WITHIN  */
  YYSYMBOL_ATMOSTONCE = 89,                /* ATMOSTONCE  */
  YYSYMBOL_SOMETIMEAFTER = 90,             /* SOMETIMEAFTER  */
  YYSYMBOL_SOMETIMEBEFORE = 91,            /* SOMETIMEBEFORE  */
  YYSYMBOL_ALWAYSWITHIN = 92,              /* ALWAYSWITHIN  */
  YYSYMBOL_HOLDDURING = 93,                /* HOLDDURING  */
  YYSYMBOL_HOLDAFTER = 94,                 /* HOLDAFTER  */
  YYSYMBOL_ISVIOLATED = 95,                /* ISVIOLATED  */
  YYSYMBOL_BOGUS = 96,                     /* BOGUS  */
  YYSYMBOL_NAME = 97,                      /* NAME  */
  YYSYMBOL_FUNCTION_SYMBOL = 98,           /* FUNCTION_SYMBOL  */
  YYSYMBOL_INTVAL = 99,                    /* INTVAL  */
  YYSYMBOL_FLOATVAL = 100,                 /* FLOATVAL  */
  YYSYMBOL_AT_TIME = 101,                  /* AT_TIME  */
  YYSYMBOL_HYPHEN = 102,                   /* HYPHEN  */
  YYSYMBOL_PLUS = 103,                     /* PLUS  */
  YYSYMBOL_MUL = 104,                      /* MUL  */
  YYSYMBOL_DIV = 105,                      /* DIV  */
  YYSYMBOL_UMINUS = 106,                   /* UMINUS  */
  YYSYMBOL_YYACCEPT = 107,                 /* $accept  */
  YYSYMBOL_mystartsymbol = 108,            /* mystartsymbol  */
  YYSYMBOL_c_domain = 109,                 /* c_domain  */
  YYSYMBOL_c_preamble = 110,               /* c_preamble  */
  YYSYMBOL_c_domain_name = 111,            /* c_domain_name  */
  YYSYMBOL_c_domain_require_def = 112,     /* c_domain_require_def  */
  YYSYMBOL_c_reqs = 113,                   /* c_reqs  */
  YYSYMBOL_c_pred_decls = 114,             /* c_pred_decls  */
  YYSYMBOL_c_pred_decl = 115,              /* c_pred_decl  */
  YYSYMBOL_c_new_pred_symbol = 116,        /* c_new_pred_symbol  */
  YYSYMBOL_c_pred_symbol = 117,            /* c_pred_symbol  */
  YYSYMBOL_c_init_pred_symbol = 118,       /* c_init_pred_symbol  */
  YYSYMBOL_c_func_decls = 119,             /* c_func_decls  */
  YYSYMBOL_c_func_decl = 120,              /* c_func_decl  */
  YYSYMBOL_c_new_func_symbol = 121,        /* c_new_func_symbol  */
  YYSYMBOL_c_typed_var_list = 122,         /* c_typed_var_list  */
  YYSYMBOL_c_var_symbol_list = 123,        /* c_var_symbol_list  */
  YYSYMBOL_c_typed_consts = 124,           /* c_typed_consts  */
  YYSYMBOL_c_const_symbols = 125,          /* c_const_symbols  */
  YYSYMBOL_c_new_const_symbols = 126,      /* c_new_const_symbols  */
  YYSYMBOL_c_typed_types = 127,            /* c_typed_types  */
  YYSYMBOL_c_parameter_symbols = 128,      /* c_parameter_symbols  */
  YYSYMBOL_c_declaration_var_symbol = 129, /* c_declaration_var_symbol  */
  YYSYMBOL_c_var_symbol = 130,             /* c_var_symbol  */
  YYSYMBOL_c_const_symbol = 131,           /* c_const_symbol  */
  YYSYMBOL_c_new_const_symbol = 132,       /* c_new_const_symbol  */
  YYSYMBOL_c_either_type = 133,            /* c_either_type  */
  YYSYMBOL_c_new_primitive_type = 134,     /* c_new_primitive_type  */
  YYSYMBOL_c_primitive_type = 135,         /* c_primitive_type  */
  YYSYMBOL_c_new_primitive_types = 136,    /* c_new_primitive_types  */
  YYSYMBOL_c_primitive_types = 137,        /* c_primitive_types  */
  YYSYMBOL_c_init_els = 138,               /* c_init_els  */
  YYSYMBOL_c_timed_initial_literal = 139,  /* c_timed_initial_literal  */
  YYSYMBOL_c_effects = 140,                /* c_effects  */
  YYSYMBOL_c_effect = 141,                 /* c_effect  */
  YYSYMBOL_c_a_effect = 142,               /* c_a_effect  */
  YYSYMBOL_c_p_effect = 143,               /* c_p_effect  */
  YYSYMBOL_c_p_effects = 144,              /* c_p_effects  */
  YYSYMBOL_c_conj_effect = 145,            /* c_conj_effect  */
  YYSYMBOL_c_da_effect = 146,              /* c_da_effect  */
  YYSYMBOL_c_da_effects = 147,             /* c_da_effects  */
  YYSYMBOL_c_timed_effect = 148,           /* c_timed_effect  */
  YYSYMBOL_c_a_effect_da = 149,            /* c_a_effect_da  */
  YYSYMBOL_c_p_effect_da = 150,            /* c_p_effect_da  */
  YYSYMBOL_c_p_effects_da = 151,           /* c_p_effects_da  */
  YYSYMBOL_c_f_assign_da = 152,            /* c_f_assign_da  */
  YYSYMBOL_c_proc_effect = 153,            /* c_proc_effect  */
  YYSYMBOL_c_proc_effects = 154,           /* c_proc_effects  */
  YYSYMBOL_c_f_exp_da = 155,               /* c_f_exp_da  */
  YYSYMBOL_c_binary_expr_da = 156,         /* c_binary_expr_da  */
  YYSYMBOL_c_duration_constraint = 157,    /* c_duration_constraint  */
  YYSYMBOL_c_d_op = 158,                   /* c_d_op  */
  YYSYMBOL_c_d_value = 159,                /* c_d_value  */
  YYSYMBOL_c_duration_constraints = 160,   /* c_duration_constraints  */
  YYSYMBOL_c_neg_simple_effect = 161,      /* c_neg_simple_effect  */
  YYSYMBOL_c_pos_simple_effect = 162,      /* c_pos_simple_effect  */
  YYSYMBOL_c_init_neg_simple_effect = 163, /* c_init_neg_simple_effect  */
  YYSYMBOL_c_init_pos_simple_effect = 164, /* c_init_pos_simple_effect  */
  YYSYMBOL_c_forall_effect = 165,          /* c_forall_effect  */
  YYSYMBOL_c_cond_effect = 166,            /* c_cond_effect  */
  YYSYMBOL_c_assignment = 167,             /* c_assignment  */
  YYSYMBOL_c_f_exp = 168,                  /* c_f_exp  */
  YYSYMBOL_c_f_exp_t = 169,                /* c_f_exp_t  */
  YYSYMBOL_c_number = 170,                 /* c_number  */
  YYSYMBOL_c_f_head = 171,                 /* c_f_head  */
  YYSYMBOL_c_ground_f_head = 172,          /* c_ground_f_head  */
  YYSYMBOL_c_comparison_op = 173,          /* c_comparison_op  */
  YYSYMBOL_c_pre_goal_descriptor = 174,    /* c_pre_goal_descriptor  */
  YYSYMBOL_c_pref_con_goal = 175,          /* c_pref_con_goal  */
  YYSYMBOL_c_pref_goal = 176,              /* c_pref_goal  */
  YYSYMBOL_c_pref_con_goal_list = 177,     /* c_pref_con_goal_list  */
  YYSYMBOL_c_pref_goal_descriptor = 178,   /* c_pref_goal_descriptor  */
  YYSYMBOL_c_constraint_goal_list = 179,   /* c_constraint_goal_list  */
  YYSYMBOL_c_constraint_goal = 180,        /* c_constraint_goal  */
  YYSYMBOL_c_goal_descriptor = 181,        /* c_goal_descriptor  */
  YYSYMBOL_c_pre_goal_descriptor_list = 182, /* c_pre_goal_descriptor_list  */
  YYSYMBOL_c_goal_list = 183,              /* c_goal_list  */
  YYSYMBOL_c_quantifier = 184,             /* c_quantifier  */
  YYSYMBOL_c_forall = 185,                 /* c_forall  */
  YYSYMBOL_c_exists = 186,                 /* c_exists  */
  YYSYMBOL_c_proposition = 187,            /* c_proposition  */
  YYSYMBOL_c_derived_proposition = 188,    /* c_derived_proposition  */
  YYSYMBOL_c_init_proposition = 189,       /* c_init_proposition  */
  YYSYMBOL_c_predicates = 190,             /* c_predicates  */
  YYSYMBOL_c_functions_def = 191,          /* c_functions_def  */
  YYSYMBOL_c_constraints_def = 192,        /* c_constraints_def  */
  YYSYMBOL_c_constraints_probdef = 193,    /* c_constraints_probdef  */
  YYSYMBOL_c_structure_defs = 194,         /* c_structure_defs  */
  YYSYMBOL_c_structure_def = 195,          /* c_structure_def  */
  YYSYMBOL_c_rule_head = 196,              /* c_rule_head  */
  YYSYMBOL_c_derivation_rule = 197,        /* c_derivation_rule  */
  YYSYMBOL_c_action_def = 198,             /* c_action_def  */
  YYSYMBOL_c_event_def = 199,              /* c_event_def  */
  YYSYMBOL_c_process_def = 200,            /* c_process_def  */
  YYSYMBOL_c_durative_action_def = 201,    /* c_durative_action_def  */
  YYSYMBOL_c_da_def_body = 202,            /* c_da_def_body  */
  YYSYMBOL_c_da_gd = 203,                  /* c_da_gd  */
  YYSYMBOL_c_da_gds = 204,                 /* c_da_gds  */
  YYSYMBOL_c_timed_gd = 205,               /* c_timed_gd  */
  YYSYMBOL_c_args_head = 206,              /* c_args_head  */
  YYSYMBOL_c_require_key = 207,            /* c_require_key  */
  YYSYMBOL_c_domain_constants = 208,       /* c_domain_constants  */
  YYSYMBOL_c_type_names = 209,             /* c_type_names  */
  YYSYMBOL_c_problem = 210,                /* c_problem  */
  YYSYMBOL_c_problem_body = 211,           /* c_problem_body  */
  YYSYMBOL_c_objects = 212,                /* c_objects  */
  YYSYMBOL_c_initial_state = 213,          /* c_initial_state  */
  YYSYMBOL_c_goals = 214,                  /* c_goals  */
  YYSYMBOL_c_goal_spec = 215,              /* c_goal_spec  */
  YYSYMBOL_c_metric_spec = 216,            /* c_metric_spec  */
  YYSYMBOL_c_length_spec = 217,            /* c_length_spec  */
  YYSYMBOL_c_optimization = 218,           /* c_optimization  */
  YYSYMBOL_c_ground_f_exp = 219,           /* c_ground_f_exp  */
  YYSYMBOL_c_binary_ground_f_exp = 220,    /* c_binary_ground_f_exp  */
  YYSYMBOL_c_binary_ground_f_pexps = 221,  /* c_binary_ground_f_pexps  */
  YYSYMBOL_c_binary_ground_f_mexps = 222,  /* c_binary_ground_f_mexps  */
  YYSYMBOL_c_plan = 223,                   /* c_plan  */
  YYSYMBOL_c_step_t_d = 224,               /* c_step_t_d  */
  YYSYMBOL_c_step_d = 225,                 /* c_step_d  */
  YYSYMBOL_c_step = 226,                   /* c_step  */
  YYSYMBOL_c_float = 227                   /* c_float  */
};
typedef enum yysymbol_kind_t yysymbol_kind_t;




#ifdef short
# undef short
#endif

/* On compilers that do not define __PTRDIFF_MAX__ etc., make sure
   <limits.h> and (if available) <stdint.h> are included
   so that the code can choose integer types of a good width.  */

#ifndef __PTRDIFF_MAX__
# include <limits.h> /* INFRINGES ON USER NAME SPACE */
# if defined __STDC_VERSION__ && 199901 <= __STDC_VERSION__
#  include <stdint.h> /* INFRINGES ON USER NAME SPACE */
#  define YY_STDINT_H
# endif
#endif

/* Narrow types that promote to a signed type and that can represent a
   signed or unsigned integer of at least N bits.  In tables they can
   save space and decrease cache pressure.  Promoting to a signed type
   helps avoid bugs in integer arithmetic.  */

#ifdef __INT_LEAST8_MAX__
typedef __INT_LEAST8_TYPE__ yytype_int8;
#elif defined YY_STDINT_H
typedef int_least8_t yytype_int8;
#else
typedef signed char yytype_int8;
#endif

#ifdef __INT_LEAST16_MAX__
typedef __INT_LEAST16_TYPE__ yytype_int16;
#elif defined YY_STDINT_H
typedef int_least16_t yytype_int16;
#else
typedef short yytype_int16;
#endif

/* Work around bug in HP-UX 11.23, which defines these macros
   incorrectly for preprocessor constants.  This workaround can likely
   be removed in 2023, as HPE has promised support for HP-UX 11.23
   (aka HP-UX 11i v2) only through the end of 2022; see Table 2 of
   <https://h20195.www2.hpe.com/V2/getpdf.aspx/4AA4-7673ENW.pdf>.  */
#ifdef __hpux
# undef UINT_LEAST8_MAX
# undef UINT_LEAST16_MAX
# define UINT_LEAST8_MAX 255
# define UINT_LEAST16_MAX 65535
#endif

#if defined __UINT_LEAST8_MAX__ && __UINT_LEAST8_MAX__ <= __INT_MAX__
typedef __UINT_LEAST8_TYPE__ yytype_uint8;
#elif (!defined __UINT_LEAST8_MAX__ && defined YY_STDINT_H \
       && UINT_LEAST8_MAX <= INT_MAX)
typedef uint_least8_t yytype_uint8;
#elif !defined __UINT_LEAST8_MAX__ && UCHAR_MAX <= INT_MAX
typedef unsigned char yytype_uint8;
#else
typedef short yytype_uint8;
#endif

#if defined __UINT_LEAST16_MAX__ && __UINT_LEAST16_MAX__ <= __INT_MAX__
typedef __UINT_LEAST16_TYPE__ yytype_uint16;
#elif (!defined __UINT_LEAST16_MAX__ && defined YY_STDINT_H \
       && UINT_LEAST16_MAX <= INT_MAX)
typedef uint_least16_t yytype_uint16;
#elif !defined __UINT_LEAST16_MAX__ && USHRT_MAX <= INT_MAX
typedef unsigned short yytype_uint16;
#else
typedef int yytype_uint16;
#endif

#ifndef YYPTRDIFF_T
# if defined __PTRDIFF_TYPE__ && defined __PTRDIFF_MAX__
#  define YYPTRDIFF_T __PTRDIFF_TYPE__
#  define YYPTRDIFF_MAXIMUM __PTRDIFF_MAX__
# elif defined PTRDIFF_MAX
#  ifndef ptrdiff_t
#   include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  endif
#  define YYPTRDIFF_T ptrdiff_t
#  define YYPTRDIFF_MAXIMUM PTRDIFF_MAX
# else
#  define YYPTRDIFF_T long
#  define YYPTRDIFF_MAXIMUM LONG_MAX
# endif
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif defined __STDC_VERSION__ && 199901 <= __STDC_VERSION__
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned
# endif
#endif

#define YYSIZE_MAXIMUM                                  \
  YY_CAST (YYPTRDIFF_T,                                 \
           (YYPTRDIFF_MAXIMUM < YY_CAST (YYSIZE_T, -1)  \
            ? YYPTRDIFF_MAXIMUM                         \
            : YY_CAST (YYSIZE_T, -1)))

#define YYSIZEOF(X) YY_CAST (YYPTRDIFF_T, sizeof (X))


/* Stored state numbers (used for stacks). */
typedef yytype_int16 yy_state_t;

/* State numbers in computations.  */
typedef int yy_state_fast_t;

#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(Msgid) dgettext ("bison-runtime", Msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(Msgid) Msgid
# endif
#endif


#ifndef YY_ATTRIBUTE_PURE
# if defined __GNUC__ && 2 < __GNUC__ + (96 <= __GNUC_MINOR__)
#  define YY_ATTRIBUTE_PURE __attribute__ ((__pure__))
# else
#  define YY_ATTRIBUTE_PURE
# endif
#endif

#ifndef YY_ATTRIBUTE_UNUSED
# if defined __GNUC__ && 2 < __GNUC__ + (7 <= __GNUC_MINOR__)
#  define YY_ATTRIBUTE_UNUSED __attribute__ ((__unused__))
# else
#  define YY_ATTRIBUTE_UNUSED
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YY_USE(E) ((void) (E))
#else
# define YY_USE(E) /* empty */
#endif

/* Suppress an incorrect diagnostic about yylval being uninitialized.  */
#if defined __GNUC__ && ! defined __ICC && 406 <= __GNUC__ * 100 + __GNUC_MINOR__
# if __GNUC__ * 100 + __GNUC_MINOR__ < 407
#  define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN                           \
    _Pragma ("GCC diagnostic push")                                     \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")
# else
#  define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN                           \
    _Pragma ("GCC diagnostic push")                                     \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")              \
    _Pragma ("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
# endif
# define YY_IGNORE_MAYBE_UNINITIALIZED_END      \
    _Pragma ("GCC diagnostic pop")
#else
# define YY_INITIAL_VALUE(Value) Value
#endif
#ifndef YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_END
#endif
#ifndef YY_INITIAL_VALUE
# define YY_INITIAL_VALUE(Value) /* Nothing. */
#endif

#if defined __cplusplus && defined __GNUC__ && ! defined __ICC && 6 <= __GNUC__
# define YY_IGNORE_USELESS_CAST_BEGIN                          \
    _Pragma ("GCC diagnostic push")                            \
    _Pragma ("GCC diagnostic ignored \"-Wuseless-cast\"")
# define YY_IGNORE_USELESS_CAST_END            \
    _Pragma ("GCC diagnostic pop")
#endif
#ifndef YY_IGNORE_USELESS_CAST_BEGIN
# define YY_IGNORE_USELESS_CAST_BEGIN
# define YY_IGNORE_USELESS_CAST_END
#endif


#define YY_ASSERT(E) ((void) (0 && (E)))

#if !defined yyoverflow

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined EXIT_SUCCESS
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
      /* Use EXIT_SUCCESS as a witness for stdlib.h.  */
#     ifndef EXIT_SUCCESS
#      define EXIT_SUCCESS 0
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's 'empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (0)
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined EXIT_SUCCESS \
       && ! ((defined YYMALLOC || defined malloc) \
             && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef EXIT_SUCCESS
#    define EXIT_SUCCESS 0
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined EXIT_SUCCESS
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined EXIT_SUCCESS
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* !defined yyoverflow */

#if (! defined yyoverflow \
     && (! defined __cplusplus \
         || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yy_state_t yyss_alloc;
  YYSTYPE yyvs_alloc;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (YYSIZEOF (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (YYSIZEOF (yy_state_t) + YYSIZEOF (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

# define YYCOPY_NEEDED 1

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack_alloc, Stack)                           \
    do                                                                  \
      {                                                                 \
        YYPTRDIFF_T yynewbytes;                                         \
        YYCOPY (&yyptr->Stack_alloc, Stack, yysize);                    \
        Stack = &yyptr->Stack_alloc;                                    \
        yynewbytes = yystacksize * YYSIZEOF (*Stack) + YYSTACK_GAP_MAXIMUM; \
        yyptr += yynewbytes / YYSIZEOF (*yyptr);                        \
      }                                                                 \
    while (0)

#endif

#if defined YYCOPY_NEEDED && YYCOPY_NEEDED
/* Copy COUNT objects from SRC to DST.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(Dst, Src, Count) \
      __builtin_memcpy (Dst, Src, YY_CAST (YYSIZE_T, (Count)) * sizeof (*(Src)))
#  else
#   define YYCOPY(Dst, Src, Count)              \
      do                                        \
        {                                       \
          YYPTRDIFF_T yyi;                      \
          for (yyi = 0; yyi < (Count); yyi++)   \
            (Dst)[yyi] = (Src)[yyi];            \
        }                                       \
      while (0)
#  endif
# endif
#endif /* !YYCOPY_NEEDED */

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  16
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   882

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  107
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  121
/* YYNRULES -- Number of rules.  */
#define YYNRULES  323
/* YYNSTATES -- Number of states.  */
#define YYNSTATES  743

/* YYMAXUTOK -- Last valid token kind.  */
#define YYMAXUTOK   361


/* YYTRANSLATE(TOKEN-NUM) -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex, with out-of-bounds checking.  */
#define YYTRANSLATE(YYX)                                \
  (0 <= (YYX) && (YYX) <= YYMAXUTOK                     \
   ? YY_CAST (yysymbol_kind_t, yytranslate[YYX])        \
   : YYSYMBOL_YYUNDEF)

/* YYTRANSLATE[TOKEN-NUM] -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex.  */
static const yytype_int8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48,    49,    50,    51,    52,    53,    54,
      55,    56,    57,    58,    59,    60,    61,    62,    63,    64,
      65,    66,    67,    68,    69,    70,    71,    72,    73,    74,
      75,    76,    77,    78,    79,    80,    81,    82,    83,    84,
      85,    86,    87,    88,    89,    90,    91,    92,    93,    94,
      95,    96,    97,    98,    99,   100,   101,   102,   103,   104,
     105,   106
};

#if YYDEBUG
/* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
static const yytype_int16 yyrline[] =
{
       0,   240,   240,   243,   247,   249,   256,   257,   258,   259,
     261,   263,   265,   268,   273,   280,   287,   288,   293,   295,
     300,   302,   310,   318,   320,   328,   333,   335,   339,   341,
     348,   361,   369,   377,   389,   391,   397,   405,   414,   419,
     420,   424,   425,   433,   440,   449,   455,   457,   459,   466,
     472,   476,   480,   484,   489,   496,   501,   503,   507,   509,
     513,   518,   520,   522,   525,   529,   535,   536,   538,   540,
     549,   550,   551,   552,   553,   557,   558,   562,   564,   566,
     573,   574,   575,   577,   581,   583,   591,   593,   601,   606,
     609,   616,   617,   621,   623,   625,   629,   633,   640,   641,
     645,   647,   649,   656,   657,   658,   660,   665,   667,   669,
     671,   673,   678,   684,   690,   695,   696,   700,   701,   703,
     704,   708,   710,   712,   714,   719,   721,   724,   727,   733,
     734,   735,   743,   747,   750,   754,   759,   766,   771,   776,
     781,   786,   788,   790,   792,   794,   799,   801,   803,   805,
     807,   809,   810,   814,   816,   818,   824,   825,   828,   831,
     833,   851,   853,   855,   861,   862,   863,   864,   865,   877,
     879,   881,   888,   890,   892,   894,   898,   903,   905,   907,
     909,   916,   919,   923,   925,   927,   932,   935,   939,   941,
     944,   946,   948,   950,   952,   954,   956,   958,   960,   962,
     967,   969,   973,   975,   978,   981,   984,   990,   993,   997,
    1000,  1004,  1005,  1009,  1016,  1023,  1028,  1033,  1038,  1040,
    1047,  1049,  1056,  1058,  1065,  1067,  1074,  1075,  1079,  1080,
    1081,  1082,  1083,  1087,  1093,  1102,  1113,  1120,  1131,  1137,
    1147,  1153,  1168,  1175,  1177,  1179,  1183,  1185,  1190,  1193,
    1197,  1199,  1201,  1203,  1208,  1213,  1218,  1219,  1221,  1222,
    1224,  1226,  1227,  1228,  1229,  1230,  1232,  1236,  1245,  1248,
    1251,  1253,  1255,  1257,  1259,  1261,  1267,  1271,  1276,  1283,
    1290,  1291,  1292,  1293,  1294,  1296,  1297,  1298,  1301,  1304,
    1307,  1310,  1314,  1316,  1323,  1326,  1330,  1337,  1338,  1343,
    1344,  1345,  1346,  1347,  1349,  1353,  1354,  1355,  1356,  1360,
    1361,  1366,  1367,  1373,  1376,  1378,  1381,  1385,  1389,  1395,
    1399,  1405,  1413,  1414
};
#endif

/** Accessing symbol of state STATE.  */
#define YY_ACCESSING_SYMBOL(State) YY_CAST (yysymbol_kind_t, yystos[State])

#if YYDEBUG || 0
/* The user-facing name of the symbol whose (internal) number is
   YYSYMBOL.  No bounds checking.  */
static const char *yysymbol_name (yysymbol_kind_t yysymbol) YY_ATTRIBUTE_UNUSED;

/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "\"end of file\"", "error", "\"invalid token\"", "OPEN_BRAC",
  "CLOSE_BRAC", "OPEN_SQ", "CLOSE_SQ", "DEFINE", "PDDLDOMAIN", "REQS",
  "EQUALITY", "STRIPS", "ADL", "NEGATIVE_PRECONDITIONS", "TYPING",
  "DISJUNCTIVE_PRECONDS", "EXT_PRECS", "UNIV_PRECS", "QUANT_PRECS",
  "COND_EFFS", "FLUENTS", "TIME", "DURATIVE_ACTIONS",
  "DURATION_INEQUALITIES", "CONTINUOUS_EFFECTS", "DERIVED_PREDICATES",
  "TIMED_INITIAL_LITERALS", "PREFERENCES", "CONSTRAINTS", "ACTION",
  "PROCESS", "EVENT", "DURATIVE_ACTION", "DERIVED", "CONSTANTS", "PREDS",
  "FUNCTIONS", "TYPES", "ARGS", "PRE", "CONDITION", "PREFERENCE",
  "START_PRE", "END_PRE", "EFFECTS", "INITIAL_EFFECT", "FINAL_EFFECT",
  "INVARIANT", "DURATION", "AT_START", "AT_END", "OVER_ALL", "AND", "OR",
  "EXISTS", "FORALL", "IMPLY", "NOT", "WHEN", "EITHER", "PROBLEM",
  "FORDOMAIN", "INITIALLY", "OBJECTS", "GOALS", "EQ", "LENGTH", "SERIAL",
  "PARALLEL", "METRIC", "MINIMIZE", "MAXIMIZE", "HASHT", "DURATION_VAR",
  "TOTAL_TIME", "INCREASE", "DECREASE", "SCALE_UP", "SCALE_DOWN", "ASSIGN",
  "GREATER", "GREATEQ", "LESS", "LESSEQ", "Q", "COLON", "ALWAYS",
  "SOMETIME", "WITHIN", "ATMOSTONCE", "SOMETIMEAFTER", "SOMETIMEBEFORE",
  "ALWAYSWITHIN", "HOLDDURING", "HOLDAFTER", "ISVIOLATED", "BOGUS", "NAME",
  "FUNCTION_SYMBOL", "INTVAL", "FLOATVAL", "AT_TIME", "HYPHEN", "PLUS",
  "MUL", "DIV", "UMINUS", "$accept", "mystartsymbol", "c_domain",
  "c_preamble", "c_domain_name", "c_domain_require_def", "c_reqs",
  "c_pred_decls", "c_pred_decl", "c_new_pred_symbol", "c_pred_symbol",
  "c_init_pred_symbol", "c_func_decls", "c_func_decl", "c_new_func_symbol",
  "c_typed_var_list", "c_var_symbol_list", "c_typed_consts",
  "c_const_symbols", "c_new_const_symbols", "c_typed_types",
  "c_parameter_symbols", "c_declaration_var_symbol", "c_var_symbol",
  "c_const_symbol", "c_new_const_symbol", "c_either_type",
  "c_new_primitive_type", "c_primitive_type", "c_new_primitive_types",
  "c_primitive_types", "c_init_els", "c_timed_initial_literal",
  "c_effects", "c_effect", "c_a_effect", "c_p_effect", "c_p_effects",
  "c_conj_effect", "c_da_effect", "c_da_effects", "c_timed_effect",
  "c_a_effect_da", "c_p_effect_da", "c_p_effects_da", "c_f_assign_da",
  "c_proc_effect", "c_proc_effects", "c_f_exp_da", "c_binary_expr_da",
  "c_duration_constraint", "c_d_op", "c_d_value", "c_duration_constraints",
  "c_neg_simple_effect", "c_pos_simple_effect", "c_init_neg_simple_effect",
  "c_init_pos_simple_effect", "c_forall_effect", "c_cond_effect",
  "c_assignment", "c_f_exp", "c_f_exp_t", "c_number", "c_f_head",
  "c_ground_f_head", "c_comparison_op", "c_pre_goal_descriptor",
  "c_pref_con_goal", "c_pref_goal", "c_pref_con_goal_list",
  "c_pref_goal_descriptor", "c_constraint_goal_list", "c_constraint_goal",
  "c_goal_descriptor", "c_pre_goal_descriptor_list", "c_goal_list",
  "c_quantifier", "c_forall", "c_exists", "c_proposition",
  "c_derived_proposition", "c_init_proposition", "c_predicates",
  "c_functions_def", "c_constraints_def", "c_constraints_probdef",
  "c_structure_defs", "c_structure_def", "c_rule_head",
  "c_derivation_rule", "c_action_def", "c_event_def", "c_process_def",
  "c_durative_action_def", "c_da_def_body", "c_da_gd", "c_da_gds",
  "c_timed_gd", "c_args_head", "c_require_key", "c_domain_constants",
  "c_type_names", "c_problem", "c_problem_body", "c_objects",
  "c_initial_state", "c_goals", "c_goal_spec", "c_metric_spec",
  "c_length_spec", "c_optimization", "c_ground_f_exp",
  "c_binary_ground_f_exp", "c_binary_ground_f_pexps",
  "c_binary_ground_f_mexps", "c_plan", "c_step_t_d", "c_step_d", "c_step",
  "c_float", YY_NULLPTR
};

static const char *
yysymbol_name (yysymbol_kind_t yysymbol)
{
  return yytname[yysymbol];
}
#endif

#define YYPACT_NINF (-524)

#define yypact_value_is_default(Yyn) \
  ((Yyn) == YYPACT_NINF)

#define YYTABLE_NINF (-70)

#define yytable_value_is_error(Yyn) \
  0

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
static const yytype_int16 yypact[] =
{
      53,    98,   -48,  -524,  -524,    55,    78,  -524,    77,  -524,
     191,   162,   261,    17,    77,    77,  -524,   262,  -524,   199,
    -524,   -29,   286,   301,   323,  -524,   328,    17,  -524,  -524,
     357,   397,  -524,   332,  -524,   676,   431,   448,   448,   448,
     448,   450,  -524,  -524,  -524,  -524,  -524,  -524,   448,   448,
    -524,  -524,   381,  -524,   473,   400,   336,    27,    38,    58,
      81,  -524,   386,   374,   298,  -524,   485,  -524,  -524,  -524,
    -524,  -524,   438,  -524,  -524,  -524,    87,  -524,   494,   542,
     499,   278,   501,   523,   497,   571,   497,   576,   497,   606,
     497,  -524,   608,   435,   386,   613,    91,   616,   605,   617,
     220,   618,   -39,   129,   620,  -524,   625,  -524,  -524,  -524,
    -524,  -524,  -524,  -524,  -524,  -524,  -524,  -524,  -524,  -524,
    -524,  -524,  -524,  -524,  -524,  -524,  -524,  -524,  -524,  -524,
    -524,   620,  -524,  -524,   620,   620,   131,   620,   620,   620,
     131,   131,   131,   629,  -524,  -524,  -524,   630,  -524,   631,
    -524,   637,  -524,   639,  -524,    33,  -524,  -524,   640,  -524,
     559,  -524,  -524,  -524,    96,  -524,  -524,  -524,  -524,    33,
    -524,  -524,  -524,   559,   544,   641,  -524,   654,   656,   233,
     660,   662,  -524,  -524,   620,   666,   620,   620,   620,   131,
     620,   559,   559,   559,   559,   559,   599,  -524,   386,   386,
    -524,   580,   674,   586,   695,  -524,   559,  -524,  -524,   696,
    -524,  -524,  -524,   620,   620,   168,  -524,  -524,  -524,  -524,
    -524,    69,   699,  -524,  -524,  -524,   642,  -524,  -524,  -524,
    -524,  -524,   710,  -524,   711,   713,   620,   620,   714,   715,
     716,   717,   718,   719,  -524,  -524,  -524,  -524,   559,  -524,
      33,  -524,   720,  -524,  -524,  -524,   253,   275,   620,   721,
     208,   412,  -524,    69,  -524,  -524,   559,   632,  -524,  -524,
    -524,   722,   723,  -524,   725,   691,   692,   693,   685,   169,
    -524,   559,   559,  -524,  -524,  -524,  -524,   730,  -524,  -524,
     638,  -524,  -524,  -524,    69,    69,    69,    69,   732,   733,
     734,  -524,  -524,   735,   737,   620,   620,   738,  -524,  -524,
    -524,  -524,  -524,  -524,  -524,   209,   210,    44,    69,    69,
      69,  -524,   620,   739,  -524,   409,   700,  -524,  -524,   701,
     702,   324,  -524,  -524,  -524,  -524,   743,   744,   745,   746,
     747,   272,   739,   739,   748,   739,   739,   739,   739,   739,
     109,  -524,   740,   750,   751,   750,   752,   753,  -524,  -524,
    -524,  -524,   673,   188,  -524,  -524,  -524,  -524,  -524,   387,
    -524,   386,  -524,   247,   190,   737,  -524,  -524,  -524,  -524,
    -524,  -524,  -524,  -524,   620,   754,   317,   559,   183,   755,
    -524,  -524,  -524,  -524,  -524,  -524,   146,   756,   757,   205,
     205,   345,   689,  -524,   760,   761,   762,   495,   763,  -524,
     377,   764,   670,   671,   767,  -524,  -524,    29,   768,   769,
    -524,  -524,  -524,   770,   405,   772,   620,   773,  -524,  -524,
     101,   101,  -524,  -524,   681,   694,  -524,  -524,    69,   306,
    -524,  -524,   267,  -524,  -524,  -524,  -524,   110,  -524,   774,
    -524,   132,  -524,  -524,  -524,  -524,  -524,  -524,   111,   775,
    -524,   533,  -524,  -524,  -524,  -524,   776,  -524,  -524,   737,
     777,   604,   778,   780,  -524,  -524,  -524,   780,   780,  -524,
     129,   781,   780,   559,   396,   375,     3,     3,   724,   726,
     782,  -524,   123,   620,   620,   620,  -524,   783,   785,   785,
    -524,   760,   101,   101,   101,   101,   101,   786,   725,   787,
     439,   559,   789,   101,  -524,  -524,  -524,  -524,   697,  -524,
     790,   687,  -524,  -524,    29,    29,    29,    29,   791,  -524,
     794,  -524,  -524,   101,   101,  -524,  -524,  -524,  -524,  -524,
     796,   797,  -524,  -524,   686,  -524,   798,   799,    69,    69,
    -524,   193,   801,   802,   803,   804,   805,   472,  -524,   619,
     806,  -524,  -524,  -524,  -524,   807,   504,   761,    50,    50,
      69,    69,    69,   559,   808,  -524,  -524,  -524,   809,   708,
     810,   131,   515,   211,   811,  -524,   812,   213,   214,    29,
      29,    29,    29,  -524,  -524,   517,    69,    69,  -524,   750,
      66,  -524,  -524,   813,   814,   815,  -524,  -524,  -524,  -524,
    -524,  -524,  -524,   101,   101,   101,   101,   101,  -524,  -524,
    -524,  -524,   816,   436,   817,   818,   819,   820,   821,   822,
     823,   824,  -524,   826,  -524,   827,  -524,  -524,  -524,  -524,
    -524,  -524,  -524,    29,  -524,    29,  -524,  -524,   -11,  -524,
    -524,  -524,  -524,   828,    69,   758,   829,   830,  -524,   539,
      47,    47,    47,    47,    47,  -524,    66,  -524,  -524,  -524,
    -524,  -524,  -524,  -524,   761,   561,   831,  -524,  -524,  -524,
    -524,   832,   833,  -524,  -524,   447,  -524,  -524,  -524,  -524,
     570,   765,   835,  -524,  -524,  -524,   836,   837,   838,   839,
      82,   840,   130,  -524,   842,  -524,  -524,  -524,    47,    47,
      47,    47,  -524,  -524,  -524,  -524,  -524,  -524,  -524,   725,
     843,   545,   559,    47,    47,    47,    47,   844,  -524,  -524,
     845,   846,   847,   848,   849,  -524,   826,  -524,  -524,  -524,
    -524,   850,  -524
};

/* YYDEFACT[STATE-NUM] -- Default reduction number in state STATE-NUM.
   Performed when YYTABLE does not specify something else to do.  Zero
   means the default is an error.  */
static const yytype_int16 yydefact[] =
{
     316,     0,     0,   323,   322,     0,     0,     3,   316,   318,
     320,     0,     0,    40,   316,   316,     1,     0,     2,     0,
     313,     0,     0,     0,     0,    51,     0,    40,   315,   314,
       0,     0,   317,     0,     5,     0,     0,     0,     0,     0,
       0,    12,   227,   232,   228,   229,   230,   231,     0,     0,
     321,    39,     0,   319,     0,     0,     0,     0,     0,     0,
       0,   233,    42,     0,     0,    57,     0,     4,     6,     9,
      10,    11,     0,   226,     8,     7,     0,    13,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,    52,     0,    38,    42,     0,     0,     0,    19,     0,
       0,     0,    45,     0,     0,   279,     0,    15,    14,   256,
     257,   267,   259,   258,   260,   261,   262,   268,   263,   264,
     266,   265,   269,   270,   271,   272,   273,   274,   275,    16,
     223,     0,   187,   213,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,   222,   236,   255,     0,   240,     0,
     238,     0,   242,     0,   276,     0,    41,   219,     0,    22,
      35,   218,    18,   221,     0,   220,    26,   277,    54,     0,
      56,    23,    24,    35,     0,     0,   200,     0,     0,     0,
       0,     0,   156,   157,     0,     0,     0,     0,     0,     0,
       0,    35,    35,    35,    35,    35,     0,    55,    42,    42,
      21,     0,     0,    33,     0,    30,    35,    57,    57,     0,
     210,   210,   214,     0,     0,   168,   164,   165,   166,   167,
      48,     0,     0,   211,   212,   234,     0,   190,   188,   186,
     191,   192,     0,   194,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,    59,    37,    36,    49,    35,    20,
       0,    29,     0,    44,    43,   216,     0,     0,     0,     0,
       0,     0,   160,     0,   151,   152,    35,     0,   193,   195,
     196,     0,     0,   199,     0,     0,     0,     0,     0,     0,
      34,    35,    35,    28,   202,   209,   203,     0,   201,   215,
       0,    46,    48,    48,     0,     0,     0,     0,     0,     0,
       0,   197,   198,     0,     0,     0,     0,     0,    53,    58,
      32,    31,   204,    50,    47,     0,     0,     0,     0,     0,
       0,   206,     0,   287,   189,     0,     0,   169,   185,     0,
       0,     0,   245,   159,   158,   146,     0,     0,     0,     0,
       0,     0,   287,   287,     0,   287,   287,   287,   287,   287,
       0,   208,     0,     0,     0,     0,     0,     0,   134,   131,
     130,   129,     0,     0,   148,   147,   149,   150,   205,     0,
      64,    42,   290,     0,     0,     0,   280,   284,   278,   281,
     282,   283,   285,   286,     0,     0,     0,    35,     0,     0,
      70,    72,    71,    74,    73,   136,     0,     0,     0,     0,
       0,     0,     0,   241,     0,     0,     0,     0,     0,   176,
       0,     0,     0,     0,     0,   297,   298,     0,     0,     0,
     183,   170,   207,     0,     0,     0,     0,     0,   235,   116,
       0,     0,   239,   237,     0,     0,   125,   133,     0,     0,
     244,   246,     0,   243,    89,    90,   225,     0,   182,     0,
     224,     0,   289,    63,    62,    61,   138,   288,     0,     0,
     293,     0,   302,   163,   301,   300,     0,   291,   184,     0,
       0,     0,     0,    69,    76,    77,    78,    69,    69,    79,
       0,     0,    69,    35,     0,     0,     0,     0,     0,     0,
       0,   132,     0,     0,     0,     0,   249,     0,     0,     0,
      92,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,    35,     0,     0,    25,    64,    48,   295,     0,   296,
       0,     0,    48,    48,     0,     0,     0,     0,     0,   292,
       0,    85,    83,     0,     0,    84,    66,    68,    67,   135,
       0,     0,   114,   115,     0,   155,     0,     0,     0,     0,
     126,     0,     0,     0,     0,     0,     0,     0,    97,     0,
       0,    99,   102,   100,   101,     0,     0,     0,     0,     0,
       0,     0,     0,    35,     0,   172,   174,   181,     0,     0,
       0,     0,     0,     0,     0,   304,     0,     0,     0,     0,
       0,     0,     0,   299,   171,     0,     0,     0,   140,     0,
       0,   112,   113,     0,     0,     0,   254,   250,   251,   252,
     247,   248,   106,     0,     0,     0,     0,     0,    93,    94,
      86,    91,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,   173,     0,   137,     0,    65,   217,   294,   303,
     162,   161,   306,   309,   305,   311,   307,   308,     0,    75,
      80,    81,    82,     0,     0,     0,     0,     0,   253,     0,
       0,     0,     0,     0,     0,    88,     0,   142,    95,   143,
      96,   144,   145,   141,     0,     0,     0,    60,   310,   312,
     139,     0,     0,   127,   128,     0,    98,   105,   103,   104,
       0,     0,     0,   117,   119,   120,     0,     0,     0,     0,
       0,     0,     0,   182,     0,   175,   153,   154,     0,     0,
       0,     0,   118,   108,   109,   110,   111,   107,    87,     0,
       0,     0,    35,     0,     0,     0,     0,     0,   177,   179,
       0,     0,     0,     0,     0,   178,     0,   122,   121,   123,
     124,     0,   180
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int16 yypgoto[] =
{
    -524,  -524,  -524,   407,  -524,   566,  -524,   759,  -524,  -524,
     690,  -524,  -524,  -524,  -524,  -171,   598,  -169,   834,   766,
     364,  -232,  -524,  -524,   192,  -524,   -63,  -524,  -154,  -524,
    -524,   340,  -524,  -227,  -350,  -524,  -524,  -524,  -524,  -523,
    -524,  -524,   359,  -524,  -524,   197,   378,  -524,  -224,  -524,
     458,   174,    70,  -524,  -336,  -328,  -524,  -524,  -337,  -322,
    -392,  -218,  -461,  -132,  -230,  -524,  -524,  -313,   496,   127,
     161,  -524,  -524,   -56,   -97,  -524,   655,  -524,   -80,  -524,
    -341,  -524,   355,  -524,  -524,  -524,  -524,  -524,   841,  -524,
    -524,  -524,  -524,  -524,  -524,  -524,  -367,  -524,  -443,   273,
    -524,  -524,  -524,  -524,   344,  -524,  -524,  -524,  -524,  -524,
    -524,  -524,  -406,  -524,   225,   224,   196,  -524,   851,  -524,
     853
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
       0,     5,     6,    36,    24,   342,    79,    97,    98,   160,
     220,   516,   100,   166,   206,   202,   203,    92,    26,    93,
     101,   260,   248,   314,   291,    94,   198,   170,   199,   102,
     279,   410,   453,   472,   389,   473,   474,   595,   390,   443,
     566,   444,   560,   561,   659,   562,   397,   484,   692,   693,
     332,   362,   490,   401,   475,   476,   454,   455,   477,   478,
     479,   491,   546,   264,   265,   465,   221,   326,   577,   676,
     510,   327,   179,   303,   328,   386,   256,   222,   427,   224,
     176,   104,   456,    38,    39,    40,   343,    41,    42,    66,
      43,    44,    45,    46,    47,   363,   440,   557,   441,   147,
     129,    48,    49,    18,   344,   345,   346,   375,   347,   348,
     349,   417,   643,   528,   644,   646,     7,     8,     9,    10,
      11
};

/* YYTABLE[YYPACT[STATE-NUM]] -- What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule whose
   number is the opposite.  If YYTABLE_NINF, syntax error.  */
static const yytype_int16 yytable[] =
{
      82,   143,   209,   263,   184,   398,   544,   175,   188,   189,
     190,   466,   395,   445,   395,   208,   393,   391,   393,   391,
     239,   240,   241,   242,   243,   392,   547,   392,    83,   245,
     246,   394,   461,   394,   178,   252,   196,   180,   181,    85,
     185,   186,   187,   621,   622,   298,   425,   261,   335,   553,
     690,    14,    15,   623,   171,    16,     1,   237,   168,    87,
     315,   316,   418,   169,   533,   534,   504,   505,   506,   261,
       3,     4,   261,   422,     2,   545,   317,   318,   319,   320,
      19,    17,    89,   395,   481,   261,   172,   232,   105,   234,
     235,   236,   158,   238,   223,   299,   282,   204,     2,   336,
     337,   338,   339,   462,   485,    12,   207,   625,   627,   605,
     310,   311,   174,    81,    25,   517,   258,   259,   589,   590,
     591,   592,   545,   229,    84,   309,   551,   463,   182,   183,
     197,   691,   395,    81,   567,    86,   395,   395,   654,   271,
     272,   395,   262,   182,   183,   262,   182,   183,   262,   182,
     183,   701,     3,     4,   682,    88,   530,   395,   395,   285,
     285,   287,   563,   563,   262,   182,   183,   262,   182,   183,
     564,   564,   -23,   308,   445,   445,     3,     4,    90,   518,
     262,   182,   183,   642,   106,   645,   647,   281,   159,   512,
     611,   414,   403,   205,   171,    13,    21,   513,   429,   262,
     486,   487,   411,   652,    20,    27,   384,   508,   329,   330,
      28,    29,   289,   333,   334,   637,   423,   640,   641,    27,
     552,   430,   431,   164,   165,   340,   172,   719,   404,   514,
     182,   183,   405,   515,   492,   424,    81,   228,   133,   645,
     425,   426,   493,   494,   495,   352,   536,    22,   171,   653,
     537,   538,   -23,   385,   395,   540,   174,   284,   395,   650,
     415,   416,   393,   391,    23,   -23,   197,   651,   497,    30,
     359,   392,   568,   569,   570,   571,   572,   394,   174,   286,
     172,    55,   445,   581,   583,   464,   360,   419,   361,    19,
     587,   588,   290,   290,   290,   290,    13,   290,   290,    99,
     369,   -27,   -27,   596,   597,    25,    25,    25,    25,    33,
      25,    25,   541,   409,   412,   413,   498,   499,   395,   500,
     325,   421,   133,   688,    34,   501,    35,   449,   131,   482,
     132,   689,    50,   133,   370,   371,   372,    80,   373,    81,
     578,   374,   502,   503,   504,   505,   506,   492,   331,   436,
     624,   626,   628,   629,   630,   493,   494,   495,   496,   149,
      52,   151,   507,   153,   134,   135,   136,   137,   138,   139,
     140,   141,   142,   356,   357,    95,   358,    96,   624,   626,
     451,   452,   655,   660,   661,   662,   663,   664,   406,   359,
     407,   509,   464,   464,   464,   464,   554,   555,   556,   396,
     542,    78,   631,    53,   -17,   360,   470,   361,   471,   -69,
     -17,   -17,   -17,   -17,   -17,   -17,   -17,   -17,   -17,   -17,
     -17,   -17,   -17,   -17,   -17,   -17,   -17,   -17,   -17,    54,
     695,   695,   695,   695,   695,    67,   681,   696,   697,   698,
     699,    76,   407,   576,    68,    69,    70,    71,   700,   635,
     350,    35,   574,    72,   409,    74,    75,   464,   464,   464,
     464,   351,   211,   212,   133,   213,   214,    57,    58,    59,
      60,    61,   292,   293,   215,   439,   610,    77,   695,   695,
     695,   695,   338,    91,   723,   724,   725,   726,   103,   216,
     217,   218,   219,   695,   695,   695,   695,   -17,   107,   731,
     732,   733,   734,   130,   425,   144,   172,   442,   620,   292,
     293,   464,   171,   464,   294,   295,   296,   297,   451,   636,
     648,   649,   613,   614,   615,   616,   617,   145,   694,   694,
     694,   694,   694,   292,   293,   146,   447,   155,   294,   295,
     666,   297,   685,   686,   172,   131,   108,   448,   407,   729,
     133,   730,   109,   110,   111,   112,   113,   114,   115,   116,
     117,   118,   119,   120,   121,   122,   123,   124,   125,   126,
     127,   253,   254,   434,   435,   148,   694,   694,   694,   694,
     150,   134,   135,   136,   137,   138,   139,   140,   141,   142,
      37,   694,   694,   694,   694,   704,   210,   211,   212,   133,
     213,   214,   702,    37,    37,    37,    37,   520,    96,   215,
     152,   131,   154,   703,    37,    37,   133,   157,   603,   604,
     161,   163,   167,   174,   216,   217,   218,   219,   521,   177,
     522,   523,   191,   192,   193,   524,   525,   526,   527,   128,
     194,   172,   195,   201,   200,   225,   720,   134,   135,   136,
     137,   138,   139,   140,   141,   142,   532,   226,   244,   133,
     227,   425,   426,   727,   230,   409,   231,   292,   293,   171,
     233,   612,   708,   709,   710,   711,   425,   247,   249,   533,
     534,   504,   505,   506,   171,    55,   376,   377,   250,   379,
     380,   381,   382,   383,   613,   614,   615,   616,   617,   251,
     255,   172,   266,   267,    56,    57,    58,    59,    60,    61,
      62,    63,    64,    65,   268,   269,   172,   270,   273,   274,
     275,   276,   277,   278,   283,   288,   301,   302,    81,   300,
     304,   305,   306,   307,   312,   313,   321,   322,   323,   324,
     325,   331,   341,   387,   353,   354,   355,   364,   365,   366,
     367,   368,   378,   388,   396,   399,   400,   402,   420,   428,
     432,   433,   438,   439,   442,   488,   446,   450,   457,   458,
     459,   460,   467,   468,   469,   480,   483,   511,   489,   519,
     529,   531,   535,   471,   586,   539,   550,   558,   559,   573,
     600,   575,   579,   173,   585,   593,   584,   548,   594,   549,
     598,   599,   601,   602,   551,   514,   606,   607,   608,   609,
     618,   619,   632,   633,   634,   638,   639,   656,   657,   658,
     665,   667,   668,   669,   670,   671,   672,   673,   674,   675,
     682,   677,   680,   683,   684,   705,   706,   707,   712,   713,
     714,   715,   716,   717,   718,   722,   280,   728,   735,   736,
     737,   738,   739,   740,   742,   582,   687,   162,   565,   437,
     156,    51,   543,   741,   721,   408,   257,   580,   678,   679,
       0,     0,     0,    32,    31,     0,     0,     0,     0,     0,
       0,     0,    73
};

static const yytype_int16 yycheck[] =
{
      56,    81,   173,   221,   136,   355,     3,   104,   140,   141,
     142,   417,   353,   405,   355,   169,   353,   353,   355,   355,
     191,   192,   193,   194,   195,   353,   487,   355,     1,   198,
     199,   353,     3,   355,   131,   206,     3,   134,   135,     1,
     137,   138,   139,   566,   567,   263,    57,     3,     4,   492,
       3,    99,   100,     3,    65,     0,     3,   189,    97,     1,
     292,   293,   375,   102,    75,    76,    77,    78,    79,     3,
      99,   100,     3,   386,    21,    72,   294,   295,   296,   297,
       3,     3,     1,   424,   425,     3,    97,   184,     1,   186,
     187,   188,     1,   190,   174,   266,   250,     1,    21,   317,
     318,   319,   320,    74,     3,     7,   169,   568,   569,   552,
     281,   282,     3,     3,    97,     4,   213,   214,   524,   525,
     526,   527,    72,   179,    97,   279,     3,    98,    99,   100,
      97,    84,   473,     3,   501,    97,   477,   478,    72,   236,
     237,   482,    98,    99,   100,    98,    99,   100,    98,    99,
     100,   674,    99,   100,    72,    97,   469,   498,   499,   256,
     257,   258,   498,   499,    98,    99,   100,    98,    99,   100,
     498,   499,     4,     4,   566,   567,    99,   100,    97,    68,
      98,    99,   100,   589,    97,   591,   592,   250,    97,    57,
     557,     1,     4,    97,    65,    97,     5,    65,    52,    98,
     430,   431,   371,   595,     8,    13,    97,    97,   305,   306,
      14,    15,     4,     4,     4,     4,   387,     4,     4,    27,
      97,    75,    76,     3,     4,   322,    97,    97,    40,    97,
      99,   100,    44,   101,    41,    52,     3,     4,    55,   645,
      57,    58,    49,    50,    51,   325,   473,    85,    65,   599,
     477,   478,    84,   350,   595,   482,     3,     4,   599,   595,
      70,    71,   599,   599,     3,    97,    97,   595,     1,     7,
      65,   599,   502,   503,   504,   505,   506,   599,     3,     4,
      97,     9,   674,   513,   516,   417,    81,   384,    83,     3,
     522,   523,    84,    84,    84,    84,    97,    84,    84,     1,
      28,     3,     4,   533,   534,    97,    97,    97,    97,     8,
      97,    97,   483,   369,    67,    68,    49,    50,   659,    52,
       3,     4,    55,   659,     1,    58,     3,   407,    50,   426,
      52,   659,     4,    55,    62,    63,    64,     1,    66,     3,
     511,    69,    75,    76,    77,    78,    79,    41,     3,     4,
     568,   569,   570,   571,   572,    49,    50,    51,    52,    86,
       3,    88,   442,    90,    86,    87,    88,    89,    90,    91,
      92,    93,    94,    49,    50,     1,    52,     3,   596,   597,
       3,     4,   600,   613,   614,   615,   616,   617,     1,    65,
       3,   447,   524,   525,   526,   527,   493,   494,   495,     3,
       4,     1,   573,     6,     4,    81,     1,    83,     3,     4,
      10,    11,    12,    13,    14,    15,    16,    17,    18,    19,
      20,    21,    22,    23,    24,    25,    26,    27,    28,    97,
     660,   661,   662,   663,   664,     4,   654,   661,   662,   663,
     664,    60,     3,     4,    37,    38,    39,    40,   666,   581,
      41,     3,   508,     3,   510,    48,    49,   589,   590,   591,
     592,    52,    53,    54,    55,    56,    57,    29,    30,    31,
      32,    33,    97,    98,    65,     3,     4,     4,   708,   709,
     710,   711,   700,    97,   708,   709,   710,   711,     3,    80,
      81,    82,    83,   723,   724,   725,   726,    97,     4,   723,
     724,   725,   726,     4,    57,     4,    97,     3,     4,    97,
      98,   643,    65,   645,   102,   103,   104,   105,     3,     4,
       3,     4,    75,    76,    77,    78,    79,     4,   660,   661,
     662,   663,   664,    97,    98,    38,    41,   102,   102,   103,
     104,   105,     3,     4,    97,    50,     4,    52,     3,     4,
      55,   722,    10,    11,    12,    13,    14,    15,    16,    17,
      18,    19,    20,    21,    22,    23,    24,    25,    26,    27,
      28,   207,   208,   399,   400,     4,   708,   709,   710,   711,
       4,    86,    87,    88,    89,    90,    91,    92,    93,    94,
      24,   723,   724,   725,   726,   675,    52,    53,    54,    55,
      56,    57,    41,    37,    38,    39,    40,    74,     3,    65,
       4,    50,     4,    52,    48,    49,    55,     4,   548,   549,
       4,     4,     4,     3,    80,    81,    82,    83,    95,     4,
      97,    98,     3,     3,     3,   102,   103,   104,   105,    97,
       3,    97,     3,    84,     4,     4,   702,    86,    87,    88,
      89,    90,    91,    92,    93,    94,    52,     3,    59,    55,
       4,    57,    58,   719,     4,   721,     4,    97,    98,    65,
       4,    52,   102,   103,   104,   105,    57,    97,     4,    75,
      76,    77,    78,    79,    65,     9,   342,   343,   102,   345,
     346,   347,   348,   349,    75,    76,    77,    78,    79,     4,
       4,    97,     3,    61,    28,    29,    30,    31,    32,    33,
      34,    35,    36,    37,     4,     4,    97,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     3,    97,
      39,    39,    39,    48,     4,    97,     4,     4,     4,     4,
       3,     3,     3,     3,    44,    44,    44,     4,     4,     4,
       4,     4,     4,     3,     3,     3,     3,    84,     4,     4,
       4,     4,    73,     3,     3,    84,     4,     4,     4,    99,
      99,     4,     4,     4,     4,     3,     3,     3,    84,     4,
       4,     4,     4,     3,    97,     4,     4,     4,     3,     3,
     104,     4,     3,   103,     4,     4,    99,    73,     4,    73,
       4,     4,     4,     4,     3,    97,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     3,
      72,     4,     4,     4,     4,     4,     4,     4,    73,     4,
       4,     4,     4,     4,     4,     3,   248,     4,     4,     4,
       4,     4,     4,     4,     4,   515,   659,    98,   499,   401,
      94,    27,   484,   736,   703,   369,   211,   512,   643,   645,
      -1,    -1,    -1,    22,    21,    -1,    -1,    -1,    -1,    -1,
      -1,    -1,    41
};

/* YYSTOS[STATE-NUM] -- The symbol kind of the accessing symbol of
   state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,     3,    21,    99,   100,   108,   109,   223,   224,   225,
     226,   227,     7,    97,    99,   100,     0,     3,   210,     3,
     223,     5,    85,     3,   111,    97,   125,   131,   223,   223,
       7,   227,   225,     8,     1,     3,   110,   112,   190,   191,
     192,   194,   195,   197,   198,   199,   200,   201,   208,   209,
       4,   125,     3,     6,    97,     9,    28,    29,    30,    31,
      32,    33,    34,    35,    36,    37,   196,     4,   110,   110,
     110,   110,     3,   195,   110,   110,    60,     4,     1,   113,
       1,     3,   180,     1,    97,     1,    97,     1,    97,     1,
      97,    97,   124,   126,   132,     1,     3,   114,   115,     1,
     119,   127,   136,     3,   188,     1,    97,     4,     4,    10,
      11,    12,    13,    14,    15,    16,    17,    18,    19,    20,
      21,    22,    23,    24,    25,    26,    27,    28,    97,   207,
       4,    50,    52,    55,    86,    87,    88,    89,    90,    91,
      92,    93,    94,   185,     4,     4,    38,   206,     4,   206,
       4,   206,     4,   206,     4,   102,   126,     4,     1,    97,
     116,     4,   114,     4,     3,     4,   120,     4,    97,   102,
     134,    65,    97,   117,     3,   181,   187,     4,   181,   179,
     181,   181,    99,   100,   170,   181,   181,   181,   170,   170,
     170,     3,     3,     3,     3,     3,     3,    97,   133,   135,
       4,    84,   122,   123,     1,    97,   121,   133,   135,   122,
      52,    53,    54,    56,    57,    65,    80,    81,    82,    83,
     117,   173,   184,   185,   186,     4,     3,     4,     4,   180,
       4,     4,   181,     4,   181,   181,   181,   170,   181,   122,
     122,   122,   122,   122,    59,   124,   124,    97,   129,     4,
     102,     4,   122,   127,   127,     4,   183,   183,   181,   181,
     128,     3,    98,   168,   170,   171,     3,    61,     4,     4,
       4,   181,   181,     4,     4,     4,     4,     4,     4,   137,
     123,   133,   135,     4,     4,   181,     4,   181,     4,     4,
      84,   131,    97,    98,   102,   103,   104,   105,   168,   122,
      97,     4,     4,   180,    39,    39,    39,    48,     4,   135,
     122,   122,     4,    97,   130,   128,   128,   168,   168,   168,
     168,     4,     4,     4,     4,     3,   174,   178,   181,   181,
     181,     3,   157,     4,     4,     4,   168,   168,   168,   168,
     181,     3,   112,   193,   211,   212,   213,   215,   216,   217,
      41,    52,   185,    44,    44,    44,    49,    50,    52,    65,
      81,    83,   158,   202,     4,     4,     4,     4,     4,    28,
      62,    63,    64,    66,    69,   214,   211,   211,     4,   211,
     211,   211,   211,   211,    97,   181,   182,     3,     3,   141,
     145,   161,   162,   165,   166,   187,     3,   153,   141,     3,
       3,   160,    84,     4,    40,    44,     1,     3,   175,   180,
     138,   124,    67,    68,     1,    70,    71,   218,   174,   181,
       4,     4,   174,   122,    52,    57,    58,   185,     4,    52,
      75,    76,     4,     4,   158,   158,     4,   157,    73,     3,
     203,   205,     3,   146,   148,   167,     4,    41,    52,   185,
       4,     3,     4,   139,   163,   164,   189,     4,    99,    99,
       4,     3,    74,    98,   170,   172,   219,     4,     4,     4,
       1,     3,   140,   142,   143,   161,   162,   165,   166,   167,
       3,   187,   181,     3,   154,     3,   171,   171,    84,    84,
     159,   168,    41,    49,    50,    51,    52,     1,    49,    50,
      52,    58,    75,    76,    77,    78,    79,   185,    97,   180,
     177,     3,    57,    65,    97,   101,   118,     4,    68,     4,
      74,    95,    97,    98,   102,   103,   104,   105,   220,     4,
     174,     4,    52,    75,    76,     4,   140,   140,   140,     4,
     140,   122,     4,   153,     3,    72,   169,   169,    73,    73,
       4,     3,    97,   205,   181,   181,   181,   204,     4,     3,
     149,   150,   152,   161,   162,   149,   147,   203,   171,   171,
     171,   171,   171,     3,   180,     4,     4,   175,   122,     3,
     189,   171,   138,   128,    99,     4,    97,   128,   128,   219,
     219,   219,   219,     4,     4,   144,   171,   171,     4,     4,
     104,     4,     4,   159,   159,   205,     4,     4,     4,     4,
       4,   203,    52,    75,    76,    77,    78,    79,     4,     4,
       4,   146,   146,     3,   168,   169,   168,   169,   168,   168,
     168,   122,     4,     4,     4,   170,     4,     4,     4,     4,
       4,     4,   219,   219,   221,   219,   222,   219,     3,     4,
     161,   162,   167,   141,    72,   168,     4,     4,     4,   151,
     171,   171,   171,   171,   171,     4,   104,     4,     4,     4,
       4,     4,     4,     4,     4,     3,   176,     4,   221,   222,
       4,   168,    72,     4,     4,     3,     4,   152,   161,   162,
       3,    84,   155,   156,   170,   171,   155,   155,   155,   155,
     168,   146,    41,    52,   185,     4,     4,     4,   102,   103,
     104,   105,    73,     4,     4,     4,     4,     4,     4,    97,
     180,   177,     3,   155,   155,   155,   155,   180,     4,     4,
     122,   155,   155,   155,   155,     4,     4,     4,     4,     4,
       4,   176,     4
};

/* YYR1[RULE-NUM] -- Symbol kind of the left-hand side of rule RULE-NUM.  */
static const yytype_uint8 yyr1[] =
{
       0,   107,   108,   108,   109,   109,   110,   110,   110,   110,
     110,   110,   110,   111,   112,   112,   113,   113,   114,   114,
     115,   115,   116,   117,   117,   118,   119,   119,   120,   120,
     121,   122,   122,   122,   123,   123,   124,   124,   124,   125,
     125,   126,   126,   127,   127,   127,   128,   128,   128,   129,
     130,   131,   132,   133,   134,   135,   136,   136,   137,   137,
     138,   138,   138,   138,   138,   139,   140,   140,   140,   140,
     141,   141,   141,   141,   141,   142,   142,   143,   143,   143,
     144,   144,   144,   144,   145,   145,   146,   146,   146,   146,
     146,   147,   147,   148,   148,   148,   148,   148,   149,   149,
     150,   150,   150,   151,   151,   151,   151,   152,   152,   152,
     152,   152,   153,   153,   153,   154,   154,   155,   155,   155,
     155,   156,   156,   156,   156,   157,   157,   157,   157,   158,
     158,   158,   159,   160,   160,   161,   162,   163,   164,   165,
     166,   167,   167,   167,   167,   167,   168,   168,   168,   168,
     168,   168,   168,   169,   169,   169,   170,   170,   171,   171,
     171,   172,   172,   172,   173,   173,   173,   173,   173,   174,
     174,   174,   175,   175,   175,   175,   175,   176,   176,   176,
     176,   177,   177,   178,   178,   178,   179,   179,   180,   180,
     180,   180,   180,   180,   180,   180,   180,   180,   180,   180,
     181,   181,   181,   181,   181,   181,   181,   182,   182,   183,
     183,   184,   184,   185,   186,   187,   188,   189,   190,   190,
     191,   191,   192,   192,   193,   193,   194,   194,   195,   195,
     195,   195,   195,   196,   197,   198,   198,   199,   199,   200,
     200,   201,   201,   202,   202,   202,   203,   203,   204,   204,
     205,   205,   205,   205,   205,   206,   207,   207,   207,   207,
     207,   207,   207,   207,   207,   207,   207,   207,   207,   207,
     207,   207,   207,   207,   207,   207,   208,   209,   210,   210,
     211,   211,   211,   211,   211,   211,   211,   211,   212,   213,
     214,   215,   216,   216,   217,   217,   217,   218,   218,   219,
     219,   219,   219,   219,   219,   220,   220,   220,   220,   221,
     221,   222,   222,   223,   223,   223,   223,   224,   224,   225,
     225,   226,   227,   227
};

/* YYR2[RULE-NUM] -- Number of symbols on the right-hand side of rule RULE-NUM.  */
static const yytype_int8 yyr2[] =
{
       0,     2,     2,     1,     5,     4,     2,     2,     2,     2,
       2,     2,     1,     4,     4,     4,     2,     0,     2,     1,
       4,     3,     1,     1,     1,     1,     2,     0,     4,     3,
       1,     4,     4,     1,     3,     0,     4,     4,     1,     2,
       0,     2,     0,     4,     4,     1,     2,     3,     0,     1,
       1,     1,     1,     4,     1,     1,     2,     0,     2,     0,
       6,     2,     2,     2,     0,     4,     2,     2,     2,     0,
       1,     1,     1,     1,     1,     4,     1,     1,     1,     1,
       2,     2,     2,     0,     4,     4,     4,     7,     5,     1,
       1,     2,     0,     4,     4,     5,     5,     3,     4,     1,
       1,     1,     1,     2,     2,     2,     0,     5,     5,     5,
       5,     5,     5,     5,     4,     2,     0,     1,     2,     1,
       1,     5,     5,     5,     5,     4,     6,     9,     9,     1,
       1,     1,     1,     2,     0,     4,     1,     4,     1,     7,
       5,     5,     5,     5,     5,     5,     4,     5,     5,     5,
       5,     1,     1,     5,     5,     1,     1,     1,     4,     4,
       1,     4,     4,     1,     1,     1,     1,     1,     1,     1,
       4,     7,     4,     5,     4,     7,     1,     4,     5,     4,
       7,     2,     0,     4,     5,     1,     2,     0,     4,     7,
       4,     4,     4,     5,     4,     5,     5,     6,     6,     5,
       1,     4,     4,     4,     5,     7,     5,     2,     0,     2,
       0,     1,     1,     1,     1,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     2,     1,     1,     1,
       1,     1,     1,     1,     5,    12,     4,    12,     4,    12,
       4,    11,     4,     3,     3,     0,     1,     4,     2,     0,
       4,     4,     4,     5,     4,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     4,     4,    12,     5,
       2,     2,     2,     2,     2,     2,     2,     0,     4,     4,
       1,     4,     5,     4,     7,     5,     5,     1,     1,     3,
       1,     1,     1,     4,     3,     3,     3,     3,     3,     1,
       2,     1,     2,     2,     3,     3,     0,     3,     1,     4,
       1,     4,     1,     1
};


enum { YYENOMEM = -2 };

#define yyerrok         (yyerrstatus = 0)
#define yyclearin       (yychar = YYEMPTY)

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab
#define YYNOMEM         goto yyexhaustedlab


#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)                                    \
  do                                                              \
    if (yychar == YYEMPTY)                                        \
      {                                                           \
        yychar = (Token);                                         \
        yylval = (Value);                                         \
        YYPOPSTACK (yylen);                                       \
        yystate = *yyssp;                                         \
        goto yybackup;                                            \
      }                                                           \
    else                                                          \
      {                                                           \
        yyerror (YY_("syntax error: cannot back up")); \
        YYERROR;                                                  \
      }                                                           \
  while (0)

/* Backward compatibility with an undocumented macro.
   Use YYerror or YYUNDEF. */
#define YYERRCODE YYUNDEF


/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)                        \
do {                                            \
  if (yydebug)                                  \
    YYFPRINTF Args;                             \
} while (0)




# define YY_SYMBOL_PRINT(Title, Kind, Value, Location)                    \
do {                                                                      \
  if (yydebug)                                                            \
    {                                                                     \
      YYFPRINTF (stderr, "%s ", Title);                                   \
      yy_symbol_print (stderr,                                            \
                  Kind, Value); \
      YYFPRINTF (stderr, "\n");                                           \
    }                                                                     \
} while (0)


/*-----------------------------------.
| Print this symbol's value on YYO.  |
`-----------------------------------*/

static void
yy_symbol_value_print (FILE *yyo,
                       yysymbol_kind_t yykind, YYSTYPE const * const yyvaluep)
{
  FILE *yyoutput = yyo;
  YY_USE (yyoutput);
  if (!yyvaluep)
    return;
  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  YY_USE (yykind);
  YY_IGNORE_MAYBE_UNINITIALIZED_END
}


/*---------------------------.
| Print this symbol on YYO.  |
`---------------------------*/

static void
yy_symbol_print (FILE *yyo,
                 yysymbol_kind_t yykind, YYSTYPE const * const yyvaluep)
{
  YYFPRINTF (yyo, "%s %s (",
             yykind < YYNTOKENS ? "token" : "nterm", yysymbol_name (yykind));

  yy_symbol_value_print (yyo, yykind, yyvaluep);
  YYFPRINTF (yyo, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

static void
yy_stack_print (yy_state_t *yybottom, yy_state_t *yytop)
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)                            \
do {                                                            \
  if (yydebug)                                                  \
    yy_stack_print ((Bottom), (Top));                           \
} while (0)


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

static void
yy_reduce_print (yy_state_t *yyssp, YYSTYPE *yyvsp,
                 int yyrule)
{
  int yylno = yyrline[yyrule];
  int yynrhs = yyr2[yyrule];
  int yyi;
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %d):\n",
             yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr,
                       YY_ACCESSING_SYMBOL (+yyssp[yyi + 1 - yynrhs]),
                       &yyvsp[(yyi + 1) - (yynrhs)]);
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)          \
do {                                    \
  if (yydebug)                          \
    yy_reduce_print (yyssp, yyvsp, Rule); \
} while (0)

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args) ((void) 0)
# define YY_SYMBOL_PRINT(Title, Kind, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif






/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

static void
yydestruct (const char *yymsg,
            yysymbol_kind_t yykind, YYSTYPE *yyvaluep)
{
  YY_USE (yyvaluep);
  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yykind, yyvaluep, yylocationp);

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  YY_USE (yykind);
  YY_IGNORE_MAYBE_UNINITIALIZED_END
}


/* Lookahead token kind.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;
/* Number of syntax errors so far.  */
int yynerrs;




/*----------.
| yyparse.  |
`----------*/

int
yyparse (void)
{
    yy_state_fast_t yystate = 0;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus = 0;

    /* Refer to the stacks through separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* Their size.  */
    YYPTRDIFF_T yystacksize = YYINITDEPTH;

    /* The state stack: array, bottom, top.  */
    yy_state_t yyssa[YYINITDEPTH];
    yy_state_t *yyss = yyssa;
    yy_state_t *yyssp = yyss;

    /* The semantic value stack: array, bottom, top.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs = yyvsa;
    YYSTYPE *yyvsp = yyvs;

  int yyn;
  /* The return value of yyparse.  */
  int yyresult;
  /* Lookahead symbol kind.  */
  yysymbol_kind_t yytoken = YYSYMBOL_YYEMPTY;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;



#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yychar = YYEMPTY; /* Cause a token to be read.  */

  goto yysetstate;


/*------------------------------------------------------------.
| yynewstate -- push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;


/*--------------------------------------------------------------------.
| yysetstate -- set current state (the top of the stack) to yystate.  |
`--------------------------------------------------------------------*/
yysetstate:
  YYDPRINTF ((stderr, "Entering state %d\n", yystate));
  YY_ASSERT (0 <= yystate && yystate < YYNSTATES);
  YY_IGNORE_USELESS_CAST_BEGIN
  *yyssp = YY_CAST (yy_state_t, yystate);
  YY_IGNORE_USELESS_CAST_END
  YY_STACK_PRINT (yyss, yyssp);

  if (yyss + yystacksize - 1 <= yyssp)
#if !defined yyoverflow && !defined YYSTACK_RELOCATE
    YYNOMEM;
#else
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYPTRDIFF_T yysize = yyssp - yyss + 1;

# if defined yyoverflow
      {
        /* Give user a chance to reallocate the stack.  Use copies of
           these so that the &'s don't force the real ones into
           memory.  */
        yy_state_t *yyss1 = yyss;
        YYSTYPE *yyvs1 = yyvs;

        /* Each stack pointer address is followed by the size of the
           data in use in that stack, in bytes.  This used to be a
           conditional around just the two extra args, but that might
           be undefined if yyoverflow is a macro.  */
        yyoverflow (YY_("memory exhausted"),
                    &yyss1, yysize * YYSIZEOF (*yyssp),
                    &yyvs1, yysize * YYSIZEOF (*yyvsp),
                    &yystacksize);
        yyss = yyss1;
        yyvs = yyvs1;
      }
# else /* defined YYSTACK_RELOCATE */
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
        YYNOMEM;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
        yystacksize = YYMAXDEPTH;

      {
        yy_state_t *yyss1 = yyss;
        union yyalloc *yyptr =
          YY_CAST (union yyalloc *,
                   YYSTACK_ALLOC (YY_CAST (YYSIZE_T, YYSTACK_BYTES (yystacksize))));
        if (! yyptr)
          YYNOMEM;
        YYSTACK_RELOCATE (yyss_alloc, yyss);
        YYSTACK_RELOCATE (yyvs_alloc, yyvs);
#  undef YYSTACK_RELOCATE
        if (yyss1 != yyssa)
          YYSTACK_FREE (yyss1);
      }
# endif

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;

      YY_IGNORE_USELESS_CAST_BEGIN
      YYDPRINTF ((stderr, "Stack size increased to %ld\n",
                  YY_CAST (long, yystacksize)));
      YY_IGNORE_USELESS_CAST_END

      if (yyss + yystacksize - 1 <= yyssp)
        YYABORT;
    }
#endif /* !defined yyoverflow && !defined YYSTACK_RELOCATE */


  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;


/*-----------.
| yybackup.  |
`-----------*/
yybackup:
  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to lookahead token.  */
  yyn = yypact[yystate];
  if (yypact_value_is_default (yyn))
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either empty, or end-of-input, or a valid lookahead.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token\n"));
      yychar = yylex ();
    }

  if (yychar <= YYEOF)
    {
      yychar = YYEOF;
      yytoken = YYSYMBOL_YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else if (yychar == YYerror)
    {
      /* The scanner already issued an error message, process directly
         to error recovery.  But do not keep the error token as
         lookahead, it is too special and may lead us to an endless
         loop in error recovery. */
      yychar = YYUNDEF;
      yytoken = YYSYMBOL_YYerror;
      goto yyerrlab1;
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yytable_value_is_error (yyn))
        goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);
  yystate = yyn;
  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END

  /* Discard the shifted token.  */
  yychar = YYEMPTY;
  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     '$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
  case 2: /* mystartsymbol: c_domain c_problem  */
#line 240 "pddl+.yacc"
                        {top_thing= (yyvsp[-1].t_domain); current_analysis->the_domain= (yyvsp[-1].t_domain); top_thing= (yyvsp[0].t_problem);
    current_analysis->the_problem= (yyvsp[0].t_problem); requires(E_TYPING);
    }
#line 2121 "pddl+.cpp"
    break;

  case 3: /* mystartsymbol: c_plan  */
#line 243 "pddl+.yacc"
              {top_thing= (yyvsp[0].t_plan); }
#line 2127 "pddl+.cpp"
    break;

  case 4: /* c_domain: OPEN_BRAC DEFINE c_domain_name c_preamble CLOSE_BRAC  */
#line 248 "pddl+.yacc"
       {(yyval.t_domain)= (yyvsp[-1].t_domain); (yyval.t_domain)->name= (yyvsp[-2].cp);delete [] (yyvsp[-2].cp);}
#line 2133 "pddl+.cpp"
    break;

  case 5: /* c_domain: OPEN_BRAC DEFINE c_domain_name error  */
#line 250 "pddl+.yacc"
        {yyerrok; (yyval.t_domain)=static_cast<domain*>(NULL);
       	log_error(E_FATAL,"Syntax error in domain"); }
#line 2140 "pddl+.cpp"
    break;

  case 6: /* c_preamble: c_domain_require_def c_preamble  */
#line 256 "pddl+.yacc"
                                      {(yyval.t_domain)= (yyvsp[0].t_domain); (yyval.t_domain)->req= (yyvsp[-1].t_pddl_req_flag);}
#line 2146 "pddl+.cpp"
    break;

  case 7: /* c_preamble: c_type_names c_preamble  */
#line 257 "pddl+.yacc"
                                      {(yyval.t_domain)= (yyvsp[0].t_domain); (yyval.t_domain)->types= (yyvsp[-1].t_type_list);}
#line 2152 "pddl+.cpp"
    break;

  case 8: /* c_preamble: c_domain_constants c_preamble  */
#line 258 "pddl+.yacc"
                                      {(yyval.t_domain)= (yyvsp[0].t_domain); (yyval.t_domain)->constants= (yyvsp[-1].t_const_symbol_list);}
#line 2158 "pddl+.cpp"
    break;

  case 9: /* c_preamble: c_predicates c_preamble  */
#line 259 "pddl+.yacc"
                                      {(yyval.t_domain)= (yyvsp[0].t_domain); 
                                       (yyval.t_domain)->predicates= (yyvsp[-1].t_pred_decl_list); }
#line 2165 "pddl+.cpp"
    break;

  case 10: /* c_preamble: c_functions_def c_preamble  */
#line 261 "pddl+.yacc"
                                      {(yyval.t_domain)= (yyvsp[0].t_domain); 
                                       (yyval.t_domain)->functions= (yyvsp[-1].t_func_decl_list); }
#line 2172 "pddl+.cpp"
    break;

  case 11: /* c_preamble: c_constraints_def c_preamble  */
#line 263 "pddl+.yacc"
                                      {(yyval.t_domain)= (yyvsp[0].t_domain);
   										(yyval.t_domain)->constraints = (yyvsp[-1].t_con_goal);}
#line 2179 "pddl+.cpp"
    break;

  case 12: /* c_preamble: c_structure_defs  */
#line 265 "pddl+.yacc"
                                      {(yyval.t_domain)= new domain((yyvsp[0].t_structure_store)); }
#line 2185 "pddl+.cpp"
    break;

  case 13: /* c_domain_name: OPEN_BRAC PDDLDOMAIN NAME CLOSE_BRAC  */
#line 268 "pddl+.yacc"
                                                     {(yyval.cp)=(yyvsp[-1].cp);}
#line 2191 "pddl+.cpp"
    break;

  case 14: /* c_domain_require_def: OPEN_BRAC REQS c_reqs CLOSE_BRAC  */
#line 274 "pddl+.yacc"
    {
	// Stash in analysis object --- we need to refer to it during parse
	//   but domain object is not created yet,
	current_analysis->req |= (yyvsp[-1].t_pddl_req_flag);
	(yyval.t_pddl_req_flag)=(yyvsp[-1].t_pddl_req_flag);
    }
#line 2202 "pddl+.cpp"
    break;

  case 15: /* c_domain_require_def: OPEN_BRAC REQS error CLOSE_BRAC  */
#line 281 "pddl+.yacc"
      {yyerrok; 
       log_error(E_FATAL,"Syntax error in requirements declaration.");
       (yyval.t_pddl_req_flag)= 0; }
#line 2210 "pddl+.cpp"
    break;

  case 16: /* c_reqs: c_reqs c_require_key  */
#line 287 "pddl+.yacc"
                         { (yyval.t_pddl_req_flag)= (yyvsp[-1].t_pddl_req_flag) | (yyvsp[0].t_pddl_req_flag); }
#line 2216 "pddl+.cpp"
    break;

  case 17: /* c_reqs: %empty  */
#line 288 "pddl+.yacc"
                         { (yyval.t_pddl_req_flag)= 0; }
#line 2222 "pddl+.cpp"
    break;

  case 18: /* c_pred_decls: c_pred_decl c_pred_decls  */
#line 294 "pddl+.yacc"
           {(yyval.t_pred_decl_list)=(yyvsp[0].t_pred_decl_list); (yyval.t_pred_decl_list)->push_front((yyvsp[-1].t_pred_decl));}
#line 2228 "pddl+.cpp"
    break;

  case 19: /* c_pred_decls: c_pred_decl  */
#line 296 "pddl+.yacc"
        {  (yyval.t_pred_decl_list)=new pred_decl_list;
           (yyval.t_pred_decl_list)->push_front((yyvsp[0].t_pred_decl)); }
#line 2235 "pddl+.cpp"
    break;

  case 20: /* c_pred_decl: OPEN_BRAC c_new_pred_symbol c_typed_var_list CLOSE_BRAC  */
#line 301 "pddl+.yacc"
       {(yyval.t_pred_decl)= new pred_decl((yyvsp[-2].t_pred_symbol),(yyvsp[-1].t_var_symbol_list),current_analysis->var_tab_stack.pop());}
#line 2241 "pddl+.cpp"
    break;

  case 21: /* c_pred_decl: OPEN_BRAC error CLOSE_BRAC  */
#line 303 "pddl+.yacc"
       {yyerrok; 
        // hope someone makes this error someday
        log_error(E_FATAL,"Syntax error in predicate declaration.");
	(yyval.t_pred_decl)= NULL; }
#line 2250 "pddl+.cpp"
    break;

  case 22: /* c_new_pred_symbol: NAME  */
#line 311 "pddl+.yacc"
         { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_put((yyvsp[0].cp));
           current_analysis->var_tab_stack.push(
           				current_analysis->buildPredTab());
           delete [] (yyvsp[0].cp); }
#line 2259 "pddl+.cpp"
    break;

  case 23: /* c_pred_symbol: EQ  */
#line 318 "pddl+.yacc"
         { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_ref("="); 
	      requires(E_EQUALITY); }
#line 2266 "pddl+.cpp"
    break;

  case 24: /* c_pred_symbol: NAME  */
#line 320 "pddl+.yacc"
         { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_get((yyvsp[0].cp)); delete [] (yyvsp[0].cp); }
#line 2272 "pddl+.cpp"
    break;

  case 25: /* c_init_pred_symbol: NAME  */
#line 328 "pddl+.yacc"
         { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_get((yyvsp[0].cp)); delete [] (yyvsp[0].cp);}
#line 2278 "pddl+.cpp"
    break;

  case 26: /* c_func_decls: c_func_decls c_func_decl  */
#line 334 "pddl+.yacc"
           {(yyval.t_func_decl_list)=(yyvsp[-1].t_func_decl_list); (yyval.t_func_decl_list)->push_back((yyvsp[0].t_func_decl));}
#line 2284 "pddl+.cpp"
    break;

  case 27: /* c_func_decls: %empty  */
#line 335 "pddl+.yacc"
                 { (yyval.t_func_decl_list)=new func_decl_list; }
#line 2290 "pddl+.cpp"
    break;

  case 28: /* c_func_decl: OPEN_BRAC c_new_func_symbol c_typed_var_list CLOSE_BRAC  */
#line 340 "pddl+.yacc"
       {(yyval.t_func_decl)= new func_decl((yyvsp[-2].t_func_symbol),(yyvsp[-1].t_var_symbol_list),current_analysis->var_tab_stack.pop());}
#line 2296 "pddl+.cpp"
    break;

  case 29: /* c_func_decl: OPEN_BRAC error CLOSE_BRAC  */
#line 342 "pddl+.yacc"
        {yyerrok; 
	 log_error(E_FATAL,"Syntax error in functor declaration.");
	 (yyval.t_func_decl)= NULL; }
#line 2304 "pddl+.cpp"
    break;

  case 30: /* c_new_func_symbol: NAME  */
#line 349 "pddl+.yacc"
         { (yyval.t_func_symbol)=current_analysis->func_tab.symbol_put((yyvsp[0].cp));
           current_analysis->var_tab_stack.push(
           		current_analysis->buildFuncTab()); 
           delete [] (yyvsp[0].cp); }
#line 2313 "pddl+.cpp"
    break;

  case 31: /* c_typed_var_list: c_var_symbol_list HYPHEN c_primitive_type c_typed_var_list  */
#line 362 "pddl+.yacc"
   {  
      (yyval.t_var_symbol_list)= (yyvsp[-3].t_var_symbol_list);
      (yyval.t_var_symbol_list)->set_types((yyvsp[-1].t_type));           /* Set types for variables */
      (yyval.t_var_symbol_list)->splice((yyval.t_var_symbol_list)->end(),*(yyvsp[0].t_var_symbol_list));   /* Join lists */ 
      delete (yyvsp[0].t_var_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
   }
#line 2325 "pddl+.cpp"
    break;

  case 32: /* c_typed_var_list: c_var_symbol_list HYPHEN c_either_type c_typed_var_list  */
#line 370 "pddl+.yacc"
   {  
      (yyval.t_var_symbol_list)= (yyvsp[-3].t_var_symbol_list);
      (yyval.t_var_symbol_list)->set_either_types((yyvsp[-1].t_type_list));    /* Set types for variables */
      (yyval.t_var_symbol_list)->splice((yyval.t_var_symbol_list)->end(),*(yyvsp[0].t_var_symbol_list));   /* Join lists */ 
      delete (yyvsp[0].t_var_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
   }
#line 2337 "pddl+.cpp"
    break;

  case 33: /* c_typed_var_list: c_var_symbol_list  */
#line 378 "pddl+.yacc"
   {
       (yyval.t_var_symbol_list)= (yyvsp[0].t_var_symbol_list);
   }
#line 2345 "pddl+.cpp"
    break;

  case 34: /* c_var_symbol_list: Q c_declaration_var_symbol c_var_symbol_list  */
#line 390 "pddl+.yacc"
     {(yyval.t_var_symbol_list)=(yyvsp[0].t_var_symbol_list); (yyvsp[0].t_var_symbol_list)->push_front((yyvsp[-1].t_var_symbol)); }
#line 2351 "pddl+.cpp"
    break;

  case 35: /* c_var_symbol_list: %empty  */
#line 391 "pddl+.yacc"
              {(yyval.t_var_symbol_list)= new var_symbol_list; }
#line 2357 "pddl+.cpp"
    break;

  case 36: /* c_typed_consts: c_new_const_symbols HYPHEN c_primitive_type c_typed_consts  */
#line 398 "pddl+.yacc"
   {  
      (yyval.t_const_symbol_list)= (yyvsp[-3].t_const_symbol_list);
      (yyvsp[-3].t_const_symbol_list)->set_types((yyvsp[-1].t_type));           /* Set types for constants */
      (yyvsp[-3].t_const_symbol_list)->splice((yyvsp[-3].t_const_symbol_list)->end(),*(yyvsp[0].t_const_symbol_list)); /* Join lists */ 
      delete (yyvsp[0].t_const_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
   }
#line 2369 "pddl+.cpp"
    break;

  case 37: /* c_typed_consts: c_new_const_symbols HYPHEN c_either_type c_typed_consts  */
#line 406 "pddl+.yacc"
   {  
      (yyval.t_const_symbol_list)= (yyvsp[-3].t_const_symbol_list);
      (yyvsp[-3].t_const_symbol_list)->set_either_types((yyvsp[-1].t_type_list));
      (yyvsp[-3].t_const_symbol_list)->splice((yyvsp[-3].t_const_symbol_list)->end(),*(yyvsp[0].t_const_symbol_list));
      delete (yyvsp[0].t_const_symbol_list);
      requires(E_TYPING);
   }
#line 2381 "pddl+.cpp"
    break;

  case 38: /* c_typed_consts: c_new_const_symbols  */
#line 414 "pddl+.yacc"
                        {(yyval.t_const_symbol_list)= (yyvsp[0].t_const_symbol_list);}
#line 2387 "pddl+.cpp"
    break;

  case 39: /* c_const_symbols: c_const_symbol c_const_symbols  */
#line 419 "pddl+.yacc"
                                  {(yyval.t_const_symbol_list)=(yyvsp[0].t_const_symbol_list); (yyvsp[0].t_const_symbol_list)->push_front((yyvsp[-1].t_const_symbol));}
#line 2393 "pddl+.cpp"
    break;

  case 40: /* c_const_symbols: %empty  */
#line 420 "pddl+.yacc"
               {(yyval.t_const_symbol_list)=new const_symbol_list;}
#line 2399 "pddl+.cpp"
    break;

  case 41: /* c_new_const_symbols: c_new_const_symbol c_new_const_symbols  */
#line 424 "pddl+.yacc"
                                          {(yyval.t_const_symbol_list)=(yyvsp[0].t_const_symbol_list); (yyvsp[0].t_const_symbol_list)->push_front((yyvsp[-1].t_const_symbol));}
#line 2405 "pddl+.cpp"
    break;

  case 42: /* c_new_const_symbols: %empty  */
#line 425 "pddl+.yacc"
               {(yyval.t_const_symbol_list)=new const_symbol_list;}
#line 2411 "pddl+.cpp"
    break;

  case 43: /* c_typed_types: c_new_primitive_types HYPHEN c_primitive_type c_typed_types  */
#line 434 "pddl+.yacc"
   {  
       (yyval.t_type_list)= (yyvsp[-3].t_type_list);
       (yyval.t_type_list)->set_types((yyvsp[-1].t_type));           /* Set types for constants */
       (yyval.t_type_list)->splice((yyval.t_type_list)->end(),*(yyvsp[0].t_type_list)); /* Join lists */ 
       delete (yyvsp[0].t_type_list);                   /* Delete (now empty) list */
   }
#line 2422 "pddl+.cpp"
    break;

  case 44: /* c_typed_types: c_new_primitive_types HYPHEN c_either_type c_typed_types  */
#line 441 "pddl+.yacc"
   {  
   // This parse needs to be excluded, we think (DPL&MF: 6/9/01)
       (yyval.t_type_list)= (yyvsp[-3].t_type_list);
       (yyval.t_type_list)->set_either_types((yyvsp[-1].t_type_list));
       (yyval.t_type_list)->splice((yyvsp[-3].t_type_list)->end(),*(yyvsp[0].t_type_list));
       delete (yyvsp[0].t_type_list);
   }
#line 2434 "pddl+.cpp"
    break;

  case 45: /* c_typed_types: c_new_primitive_types  */
#line 450 "pddl+.yacc"
      { (yyval.t_type_list)= (yyvsp[0].t_type_list); }
#line 2440 "pddl+.cpp"
    break;

  case 46: /* c_parameter_symbols: c_parameter_symbols c_const_symbol  */
#line 456 "pddl+.yacc"
         {(yyval.t_parameter_symbol_list)=(yyvsp[-1].t_parameter_symbol_list); (yyval.t_parameter_symbol_list)->push_back((yyvsp[0].t_const_symbol)); }
#line 2446 "pddl+.cpp"
    break;

  case 47: /* c_parameter_symbols: c_parameter_symbols Q c_var_symbol  */
#line 458 "pddl+.yacc"
         {(yyval.t_parameter_symbol_list)=(yyvsp[-2].t_parameter_symbol_list); (yyval.t_parameter_symbol_list)->push_back((yyvsp[0].t_var_symbol)); }
#line 2452 "pddl+.cpp"
    break;

  case 48: /* c_parameter_symbols: %empty  */
#line 459 "pddl+.yacc"
                {(yyval.t_parameter_symbol_list)= new parameter_symbol_list;}
#line 2458 "pddl+.cpp"
    break;

  case 49: /* c_declaration_var_symbol: NAME  */
#line 466 "pddl+.yacc"
         { (yyval.t_var_symbol)= current_analysis->var_tab_stack.top()->symbol_put((yyvsp[0].cp)); delete [] (yyvsp[0].cp); }
#line 2464 "pddl+.cpp"
    break;

  case 50: /* c_var_symbol: NAME  */
#line 472 "pddl+.yacc"
         { (yyval.t_var_symbol)= current_analysis->var_tab_stack.symbol_get((yyvsp[0].cp)); delete [] (yyvsp[0].cp); }
#line 2470 "pddl+.cpp"
    break;

  case 51: /* c_const_symbol: NAME  */
#line 476 "pddl+.yacc"
         { (yyval.t_const_symbol)= current_analysis->const_tab.symbol_get((yyvsp[0].cp)); delete [] (yyvsp[0].cp); }
#line 2476 "pddl+.cpp"
    break;

  case 52: /* c_new_const_symbol: NAME  */
#line 480 "pddl+.yacc"
         { (yyval.t_const_symbol)= current_analysis->const_tab.symbol_put((yyvsp[0].cp)); delete [] (yyvsp[0].cp);}
#line 2482 "pddl+.cpp"
    break;

  case 53: /* c_either_type: OPEN_BRAC EITHER c_primitive_types CLOSE_BRAC  */
#line 485 "pddl+.yacc"
     { (yyval.t_type_list)= (yyvsp[-1].t_type_list); }
#line 2488 "pddl+.cpp"
    break;

  case 54: /* c_new_primitive_type: NAME  */
#line 490 "pddl+.yacc"
     { (yyval.t_type)= current_analysis->pddl_type_tab.symbol_ref((yyvsp[0].cp)); delete [] (yyvsp[0].cp);}
#line 2494 "pddl+.cpp"
    break;

  case 55: /* c_primitive_type: NAME  */
#line 497 "pddl+.yacc"
     { (yyval.t_type)= current_analysis->pddl_type_tab.symbol_ref((yyvsp[0].cp)); delete [] (yyvsp[0].cp);}
#line 2500 "pddl+.cpp"
    break;

  case 56: /* c_new_primitive_types: c_new_primitive_types c_new_primitive_type  */
#line 502 "pddl+.yacc"
        {(yyval.t_type_list)= (yyvsp[-1].t_type_list); (yyval.t_type_list)->push_back((yyvsp[0].t_type));}
#line 2506 "pddl+.cpp"
    break;

  case 57: /* c_new_primitive_types: %empty  */
#line 503 "pddl+.yacc"
                {(yyval.t_type_list)= new pddl_type_list;}
#line 2512 "pddl+.cpp"
    break;

  case 58: /* c_primitive_types: c_primitive_types c_primitive_type  */
#line 508 "pddl+.yacc"
        {(yyval.t_type_list)= (yyvsp[-1].t_type_list); (yyval.t_type_list)->push_back((yyvsp[0].t_type));}
#line 2518 "pddl+.cpp"
    break;

  case 59: /* c_primitive_types: %empty  */
#line 509 "pddl+.yacc"
                {(yyval.t_type_list)= new pddl_type_list;}
#line 2524 "pddl+.cpp"
    break;

  case 60: /* c_init_els: c_init_els OPEN_BRAC EQ c_f_head c_number CLOSE_BRAC  */
#line 514 "pddl+.yacc"
        { (yyval.t_effect_lists)=(yyvsp[-5].t_effect_lists);
	  (yyval.t_effect_lists)->assign_effects.push_back(new assignment((yyvsp[-2].t_func_term),E_ASSIGN,(yyvsp[-1].t_num_expression)));  
          requires(E_FLUENTS); 
	}
#line 2533 "pddl+.cpp"
    break;

  case 61: /* c_init_els: c_init_els c_init_pos_simple_effect  */
#line 519 "pddl+.yacc"
        { (yyval.t_effect_lists)=(yyvsp[-1].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[0].t_simple_effect)); }
#line 2539 "pddl+.cpp"
    break;

  case 62: /* c_init_els: c_init_els c_init_neg_simple_effect  */
#line 521 "pddl+.yacc"
        { (yyval.t_effect_lists)=(yyvsp[-1].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[0].t_simple_effect)); }
#line 2545 "pddl+.cpp"
    break;

  case 63: /* c_init_els: c_init_els c_timed_initial_literal  */
#line 523 "pddl+.yacc"
                { (yyval.t_effect_lists)=(yyvsp[-1].t_effect_lists); (yyval.t_effect_lists)->timed_effects.push_back((yyvsp[0].t_timed_effect)); }
#line 2551 "pddl+.cpp"
    break;

  case 64: /* c_init_els: %empty  */
#line 525 "pddl+.yacc"
        { (yyval.t_effect_lists)= new effect_lists;}
#line 2557 "pddl+.cpp"
    break;

  case 65: /* c_timed_initial_literal: OPEN_BRAC AT_TIME c_init_els CLOSE_BRAC  */
#line 530 "pddl+.yacc"
   { requires(E_TIMED_INITIAL_LITERALS); 
   		(yyval.t_timed_effect)=new timed_initial_literal((yyvsp[-1].t_effect_lists),(yyvsp[-2].fval));}
#line 2564 "pddl+.cpp"
    break;

  case 66: /* c_effects: c_a_effect c_effects  */
#line 535 "pddl+.yacc"
                                  {(yyval.t_effect_lists)=(yyvsp[0].t_effect_lists); (yyval.t_effect_lists)->append_effects((yyvsp[-1].t_effect_lists)); delete (yyvsp[-1].t_effect_lists);}
#line 2570 "pddl+.cpp"
    break;

  case 67: /* c_effects: c_cond_effect c_effects  */
#line 536 "pddl+.yacc"
                                  {(yyval.t_effect_lists)=(yyvsp[0].t_effect_lists); (yyval.t_effect_lists)->cond_effects.push_front((yyvsp[-1].t_cond_effect)); 
                                      requires(E_COND_EFFS);}
#line 2577 "pddl+.cpp"
    break;

  case 68: /* c_effects: c_forall_effect c_effects  */
#line 538 "pddl+.yacc"
                                  {(yyval.t_effect_lists)=(yyvsp[0].t_effect_lists); (yyval.t_effect_lists)->forall_effects.push_front((yyvsp[-1].t_forall_effect));
                                      requires(E_COND_EFFS);}
#line 2584 "pddl+.cpp"
    break;

  case 69: /* c_effects: %empty  */
#line 540 "pddl+.yacc"
                                  {(yyval.t_effect_lists)=new effect_lists(); }
#line 2590 "pddl+.cpp"
    break;

  case 70: /* c_effect: c_conj_effect  */
#line 549 "pddl+.yacc"
                        {(yyval.t_effect_lists)= (yyvsp[0].t_effect_lists);}
#line 2596 "pddl+.cpp"
    break;

  case 71: /* c_effect: c_pos_simple_effect  */
#line 550 "pddl+.yacc"
                        {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[0].t_simple_effect));}
#line 2602 "pddl+.cpp"
    break;

  case 72: /* c_effect: c_neg_simple_effect  */
#line 551 "pddl+.yacc"
                        {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[0].t_simple_effect));}
#line 2608 "pddl+.cpp"
    break;

  case 73: /* c_effect: c_cond_effect  */
#line 552 "pddl+.yacc"
                        {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->cond_effects.push_front((yyvsp[0].t_cond_effect));}
#line 2614 "pddl+.cpp"
    break;

  case 74: /* c_effect: c_forall_effect  */
#line 553 "pddl+.yacc"
                        {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->forall_effects.push_front((yyvsp[0].t_forall_effect));}
#line 2620 "pddl+.cpp"
    break;

  case 75: /* c_a_effect: OPEN_BRAC AND c_p_effects CLOSE_BRAC  */
#line 557 "pddl+.yacc"
                                         {(yyval.t_effect_lists)= (yyvsp[-1].t_effect_lists);}
#line 2626 "pddl+.cpp"
    break;

  case 76: /* c_a_effect: c_p_effect  */
#line 558 "pddl+.yacc"
                      {(yyval.t_effect_lists)= (yyvsp[0].t_effect_lists);}
#line 2632 "pddl+.cpp"
    break;

  case 77: /* c_p_effect: c_neg_simple_effect  */
#line 563 "pddl+.yacc"
        {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[0].t_simple_effect));}
#line 2638 "pddl+.cpp"
    break;

  case 78: /* c_p_effect: c_pos_simple_effect  */
#line 565 "pddl+.yacc"
        {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[0].t_simple_effect));}
#line 2644 "pddl+.cpp"
    break;

  case 79: /* c_p_effect: c_assignment  */
#line 567 "pddl+.yacc"
        {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[0].t_assignment));
         requires(E_FLUENTS);}
#line 2651 "pddl+.cpp"
    break;

  case 80: /* c_p_effects: c_p_effects c_neg_simple_effect  */
#line 573 "pddl+.yacc"
                                    {(yyval.t_effect_lists)= (yyvsp[-1].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[0].t_simple_effect));}
#line 2657 "pddl+.cpp"
    break;

  case 81: /* c_p_effects: c_p_effects c_pos_simple_effect  */
#line 574 "pddl+.yacc"
                                    {(yyval.t_effect_lists)= (yyvsp[-1].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[0].t_simple_effect));}
#line 2663 "pddl+.cpp"
    break;

  case 82: /* c_p_effects: c_p_effects c_assignment  */
#line 575 "pddl+.yacc"
                                    {(yyval.t_effect_lists)= (yyvsp[-1].t_effect_lists); (yyval.t_effect_lists)->assign_effects.push_back((yyvsp[0].t_assignment));
                                     requires(E_FLUENTS); }
#line 2670 "pddl+.cpp"
    break;

  case 83: /* c_p_effects: %empty  */
#line 577 "pddl+.yacc"
                 { (yyval.t_effect_lists)= new effect_lists; }
#line 2676 "pddl+.cpp"
    break;

  case 84: /* c_conj_effect: OPEN_BRAC AND c_effects CLOSE_BRAC  */
#line 582 "pddl+.yacc"
        { (yyval.t_effect_lists)=(yyvsp[-1].t_effect_lists); }
#line 2682 "pddl+.cpp"
    break;

  case 85: /* c_conj_effect: OPEN_BRAC AND error CLOSE_BRAC  */
#line 584 "pddl+.yacc"
        {yyerrok; (yyval.t_effect_lists)=NULL;
	 log_error(E_FATAL,"Syntax error in (and ...)");
	}
#line 2690 "pddl+.cpp"
    break;

  case 86: /* c_da_effect: OPEN_BRAC AND c_da_effects CLOSE_BRAC  */
#line 592 "pddl+.yacc"
        { (yyval.t_effect_lists)=(yyvsp[-1].t_effect_lists); }
#line 2696 "pddl+.cpp"
    break;

  case 87: /* c_da_effect: OPEN_BRAC c_forall OPEN_BRAC c_typed_var_list CLOSE_BRAC c_da_effect CLOSE_BRAC  */
#line 597 "pddl+.yacc"
        { (yyval.t_effect_lists)= new effect_lists; 
          (yyval.t_effect_lists)->forall_effects.push_back(
	       new forall_effect((yyvsp[-1].t_effect_lists), (yyvsp[-3].t_var_symbol_list), current_analysis->var_tab_stack.pop())); 
          requires(E_COND_EFFS);}
#line 2705 "pddl+.cpp"
    break;

  case 88: /* c_da_effect: OPEN_BRAC WHEN c_da_gd c_da_effect CLOSE_BRAC  */
#line 602 "pddl+.yacc"
        { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->cond_effects.push_back(
	       new cond_effect((yyvsp[-2].t_goal),(yyvsp[-1].t_effect_lists)));
          requires(E_COND_EFFS); }
#line 2714 "pddl+.cpp"
    break;

  case 89: /* c_da_effect: c_timed_effect  */
#line 607 "pddl+.yacc"
        { (yyval.t_effect_lists)=new effect_lists;
          (yyval.t_effect_lists)->timed_effects.push_back((yyvsp[0].t_timed_effect)); }
#line 2721 "pddl+.cpp"
    break;

  case 90: /* c_da_effect: c_assignment  */
#line 610 "pddl+.yacc"
        { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[0].t_assignment));
          requires(E_FLUENTS); }
#line 2729 "pddl+.cpp"
    break;

  case 91: /* c_da_effects: c_da_effects c_da_effect  */
#line 616 "pddl+.yacc"
                             { (yyval.t_effect_lists)=(yyvsp[-1].t_effect_lists); (yyvsp[-1].t_effect_lists)->append_effects((yyvsp[0].t_effect_lists)); delete (yyvsp[0].t_effect_lists); }
#line 2735 "pddl+.cpp"
    break;

  case 92: /* c_da_effects: %empty  */
#line 617 "pddl+.yacc"
                { (yyval.t_effect_lists)= new effect_lists; }
#line 2741 "pddl+.cpp"
    break;

  case 93: /* c_timed_effect: OPEN_BRAC AT_START c_a_effect_da CLOSE_BRAC  */
#line 622 "pddl+.yacc"
        {(yyval.t_timed_effect)=new timed_effect((yyvsp[-1].t_effect_lists),E_AT_START);}
#line 2747 "pddl+.cpp"
    break;

  case 94: /* c_timed_effect: OPEN_BRAC AT_END c_a_effect_da CLOSE_BRAC  */
#line 624 "pddl+.yacc"
        {(yyval.t_timed_effect)=new timed_effect((yyvsp[-1].t_effect_lists),E_AT_END);}
#line 2753 "pddl+.cpp"
    break;

  case 95: /* c_timed_effect: OPEN_BRAC INCREASE c_f_head c_f_exp_t CLOSE_BRAC  */
#line 626 "pddl+.yacc"
        {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[-2].t_func_term),E_INCREASE,(yyvsp[-1].t_expression))); }
#line 2761 "pddl+.cpp"
    break;

  case 96: /* c_timed_effect: OPEN_BRAC DECREASE c_f_head c_f_exp_t CLOSE_BRAC  */
#line 630 "pddl+.yacc"
        {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[-2].t_func_term),E_DECREASE,(yyvsp[-1].t_expression))); }
#line 2769 "pddl+.cpp"
    break;

  case 97: /* c_timed_effect: OPEN_BRAC error CLOSE_BRAC  */
#line 634 "pddl+.yacc"
        {yyerrok; (yyval.t_timed_effect)=NULL;
	log_error(E_FATAL,"Syntax error in timed effect"); }
#line 2776 "pddl+.cpp"
    break;

  case 98: /* c_a_effect_da: OPEN_BRAC AND c_p_effects_da CLOSE_BRAC  */
#line 640 "pddl+.yacc"
                                            {(yyval.t_effect_lists)= (yyvsp[-1].t_effect_lists);}
#line 2782 "pddl+.cpp"
    break;

  case 99: /* c_a_effect_da: c_p_effect_da  */
#line 641 "pddl+.yacc"
                         {(yyval.t_effect_lists)= (yyvsp[0].t_effect_lists);}
#line 2788 "pddl+.cpp"
    break;

  case 100: /* c_p_effect_da: c_neg_simple_effect  */
#line 646 "pddl+.yacc"
        {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[0].t_simple_effect));}
#line 2794 "pddl+.cpp"
    break;

  case 101: /* c_p_effect_da: c_pos_simple_effect  */
#line 648 "pddl+.yacc"
        {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[0].t_simple_effect));}
#line 2800 "pddl+.cpp"
    break;

  case 102: /* c_p_effect_da: c_f_assign_da  */
#line 650 "pddl+.yacc"
        {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[0].t_assignment));
         requires(E_FLUENTS);}
#line 2807 "pddl+.cpp"
    break;

  case 103: /* c_p_effects_da: c_p_effects_da c_neg_simple_effect  */
#line 656 "pddl+.yacc"
                                       {(yyval.t_effect_lists)= (yyvsp[-1].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[0].t_simple_effect));}
#line 2813 "pddl+.cpp"
    break;

  case 104: /* c_p_effects_da: c_p_effects_da c_pos_simple_effect  */
#line 657 "pddl+.yacc"
                                       {(yyval.t_effect_lists)= (yyvsp[-1].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[0].t_simple_effect));}
#line 2819 "pddl+.cpp"
    break;

  case 105: /* c_p_effects_da: c_p_effects_da c_f_assign_da  */
#line 658 "pddl+.yacc"
                                       {(yyval.t_effect_lists)= (yyvsp[-1].t_effect_lists); (yyval.t_effect_lists)->assign_effects.push_back((yyvsp[0].t_assignment));
                                     requires(E_FLUENTS); }
#line 2826 "pddl+.cpp"
    break;

  case 106: /* c_p_effects_da: %empty  */
#line 660 "pddl+.yacc"
                 { (yyval.t_effect_lists)= new effect_lists; }
#line 2832 "pddl+.cpp"
    break;

  case 107: /* c_f_assign_da: OPEN_BRAC ASSIGN c_f_head c_f_exp_da CLOSE_BRAC  */
#line 666 "pddl+.yacc"
     { (yyval.t_assignment)= new assignment((yyvsp[-2].t_func_term),E_ASSIGN,(yyvsp[-1].t_expression)); }
#line 2838 "pddl+.cpp"
    break;

  case 108: /* c_f_assign_da: OPEN_BRAC INCREASE c_f_head c_f_exp_da CLOSE_BRAC  */
#line 668 "pddl+.yacc"
     { (yyval.t_assignment)= new assignment((yyvsp[-2].t_func_term),E_INCREASE,(yyvsp[-1].t_expression)); }
#line 2844 "pddl+.cpp"
    break;

  case 109: /* c_f_assign_da: OPEN_BRAC DECREASE c_f_head c_f_exp_da CLOSE_BRAC  */
#line 670 "pddl+.yacc"
     { (yyval.t_assignment)= new assignment((yyvsp[-2].t_func_term),E_DECREASE,(yyvsp[-1].t_expression)); }
#line 2850 "pddl+.cpp"
    break;

  case 110: /* c_f_assign_da: OPEN_BRAC SCALE_UP c_f_head c_f_exp_da CLOSE_BRAC  */
#line 672 "pddl+.yacc"
     { (yyval.t_assignment)= new assignment((yyvsp[-2].t_func_term),E_SCALE_UP,(yyvsp[-1].t_expression)); }
#line 2856 "pddl+.cpp"
    break;

  case 111: /* c_f_assign_da: OPEN_BRAC SCALE_DOWN c_f_head c_f_exp_da CLOSE_BRAC  */
#line 674 "pddl+.yacc"
     { (yyval.t_assignment)= new assignment((yyvsp[-2].t_func_term),E_SCALE_DOWN,(yyvsp[-1].t_expression)); }
#line 2862 "pddl+.cpp"
    break;

  case 112: /* c_proc_effect: OPEN_BRAC INCREASE c_f_head c_f_exp_t CLOSE_BRAC  */
#line 679 "pddl+.yacc"
        {(yyval.t_effect_lists)=new effect_lists; 
         timed_effect * te = new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_effect_lists)->timed_effects.push_front(te);
         te->effs->assign_effects.push_front(
	     new assignment((yyvsp[-2].t_func_term),E_INCREASE,(yyvsp[-1].t_expression))); }
#line 2872 "pddl+.cpp"
    break;

  case 113: /* c_proc_effect: OPEN_BRAC DECREASE c_f_head c_f_exp_t CLOSE_BRAC  */
#line 685 "pddl+.yacc"
        {(yyval.t_effect_lists)=new effect_lists; 
         timed_effect * te = new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_effect_lists)->timed_effects.push_front(te);
         te->effs->assign_effects.push_front(
	     new assignment((yyvsp[-2].t_func_term),E_DECREASE,(yyvsp[-1].t_expression))); }
#line 2882 "pddl+.cpp"
    break;

  case 114: /* c_proc_effect: OPEN_BRAC AND c_proc_effects CLOSE_BRAC  */
#line 691 "pddl+.yacc"
                {(yyval.t_effect_lists) = (yyvsp[-1].t_effect_lists);}
#line 2888 "pddl+.cpp"
    break;

  case 115: /* c_proc_effects: c_proc_effects c_proc_effect  */
#line 695 "pddl+.yacc"
                               { (yyval.t_effect_lists)=(yyvsp[-1].t_effect_lists); (yyvsp[-1].t_effect_lists)->append_effects((yyvsp[0].t_effect_lists)); delete (yyvsp[0].t_effect_lists); }
#line 2894 "pddl+.cpp"
    break;

  case 116: /* c_proc_effects: %empty  */
#line 696 "pddl+.yacc"
                { (yyval.t_effect_lists)= new effect_lists; }
#line 2900 "pddl+.cpp"
    break;

  case 117: /* c_f_exp_da: c_binary_expr_da  */
#line 700 "pddl+.yacc"
                     {(yyval.t_expression)= (yyvsp[0].t_expression);}
#line 2906 "pddl+.cpp"
    break;

  case 118: /* c_f_exp_da: Q DURATION_VAR  */
#line 701 "pddl+.yacc"
                   {(yyval.t_expression)= new special_val_expr(E_DURATION_VAR);
                    requires( E_DURATION_INEQUALITIES );}
#line 2913 "pddl+.cpp"
    break;

  case 119: /* c_f_exp_da: c_number  */
#line 703 "pddl+.yacc"
             { (yyval.t_expression)=(yyvsp[0].t_num_expression); }
#line 2919 "pddl+.cpp"
    break;

  case 120: /* c_f_exp_da: c_f_head  */
#line 704 "pddl+.yacc"
              { (yyval.t_expression)= (yyvsp[0].t_func_term); }
#line 2925 "pddl+.cpp"
    break;

  case 121: /* c_binary_expr_da: OPEN_BRAC PLUS c_f_exp_da c_f_exp_da CLOSE_BRAC  */
#line 709 "pddl+.yacc"
        { (yyval.t_expression)= new plus_expression((yyvsp[-2].t_expression),(yyvsp[-1].t_expression)); }
#line 2931 "pddl+.cpp"
    break;

  case 122: /* c_binary_expr_da: OPEN_BRAC HYPHEN c_f_exp_da c_f_exp_da CLOSE_BRAC  */
#line 711 "pddl+.yacc"
        { (yyval.t_expression)= new minus_expression((yyvsp[-2].t_expression),(yyvsp[-1].t_expression)); }
#line 2937 "pddl+.cpp"
    break;

  case 123: /* c_binary_expr_da: OPEN_BRAC MUL c_f_exp_da c_f_exp_da CLOSE_BRAC  */
#line 713 "pddl+.yacc"
        { (yyval.t_expression)= new mul_expression((yyvsp[-2].t_expression),(yyvsp[-1].t_expression)); }
#line 2943 "pddl+.cpp"
    break;

  case 124: /* c_binary_expr_da: OPEN_BRAC DIV c_f_exp_da c_f_exp_da CLOSE_BRAC  */
#line 715 "pddl+.yacc"
        { (yyval.t_expression)= new div_expression((yyvsp[-2].t_expression),(yyvsp[-1].t_expression)); }
#line 2949 "pddl+.cpp"
    break;

  case 125: /* c_duration_constraint: OPEN_BRAC AND c_duration_constraints CLOSE_BRAC  */
#line 720 "pddl+.yacc"
        { (yyval.t_goal)= new conj_goal((yyvsp[-1].t_goal_list)); }
#line 2955 "pddl+.cpp"
    break;

  case 126: /* c_duration_constraint: OPEN_BRAC c_d_op Q DURATION_VAR c_d_value CLOSE_BRAC  */
#line 722 "pddl+.yacc"
        { (yyval.t_goal)= new timed_goal(new comparison((yyvsp[-4].t_comparison_op),
        			new special_val_expr(E_DURATION_VAR),(yyvsp[-1].t_expression)),E_AT_START); }
#line 2962 "pddl+.cpp"
    break;

  case 127: /* c_duration_constraint: OPEN_BRAC AT_START OPEN_BRAC c_d_op Q DURATION_VAR c_d_value CLOSE_BRAC CLOSE_BRAC  */
#line 725 "pddl+.yacc"
                { (yyval.t_goal) = new timed_goal(new comparison((yyvsp[-5].t_comparison_op),
					new special_val_expr(E_DURATION_VAR),(yyvsp[-2].t_expression)),E_AT_START);}
#line 2969 "pddl+.cpp"
    break;

  case 128: /* c_duration_constraint: OPEN_BRAC AT_END OPEN_BRAC c_d_op Q DURATION_VAR c_d_value CLOSE_BRAC CLOSE_BRAC  */
#line 728 "pddl+.yacc"
                { (yyval.t_goal) = new timed_goal(new comparison((yyvsp[-5].t_comparison_op),
					new special_val_expr(E_DURATION_VAR),(yyvsp[-2].t_expression)),E_AT_END);}
#line 2976 "pddl+.cpp"
    break;

  case 129: /* c_d_op: LESSEQ  */
#line 733 "pddl+.yacc"
             {(yyval.t_comparison_op)= E_LESSEQ; requires(E_DURATION_INEQUALITIES);}
#line 2982 "pddl+.cpp"
    break;

  case 130: /* c_d_op: GREATEQ  */
#line 734 "pddl+.yacc"
             {(yyval.t_comparison_op)= E_GREATEQ; requires(E_DURATION_INEQUALITIES);}
#line 2988 "pddl+.cpp"
    break;

  case 131: /* c_d_op: EQ  */
#line 735 "pddl+.yacc"
             {(yyval.t_comparison_op)= E_EQUALS; }
#line 2994 "pddl+.cpp"
    break;

  case 132: /* c_d_value: c_f_exp  */
#line 743 "pddl+.yacc"
             {(yyval.t_expression)= (yyvsp[0].t_expression); }
#line 3000 "pddl+.cpp"
    break;

  case 133: /* c_duration_constraints: c_duration_constraints c_duration_constraint  */
#line 748 "pddl+.yacc"
        { (yyval.t_goal_list)=(yyvsp[-1].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[0].t_goal)); }
#line 3006 "pddl+.cpp"
    break;

  case 134: /* c_duration_constraints: %empty  */
#line 750 "pddl+.yacc"
        { (yyval.t_goal_list)= new goal_list; }
#line 3012 "pddl+.cpp"
    break;

  case 135: /* c_neg_simple_effect: OPEN_BRAC NOT c_proposition CLOSE_BRAC  */
#line 755 "pddl+.yacc"
     { (yyval.t_simple_effect)= new simple_effect((yyvsp[-1].t_proposition)); }
#line 3018 "pddl+.cpp"
    break;

  case 136: /* c_pos_simple_effect: c_proposition  */
#line 760 "pddl+.yacc"
     { (yyval.t_simple_effect)= new simple_effect((yyvsp[0].t_proposition)); }
#line 3024 "pddl+.cpp"
    break;

  case 137: /* c_init_neg_simple_effect: OPEN_BRAC NOT c_init_proposition CLOSE_BRAC  */
#line 767 "pddl+.yacc"
     { (yyval.t_simple_effect)= new simple_effect((yyvsp[-1].t_proposition)); }
#line 3030 "pddl+.cpp"
    break;

  case 138: /* c_init_pos_simple_effect: c_init_proposition  */
#line 772 "pddl+.yacc"
     { (yyval.t_simple_effect)= new simple_effect((yyvsp[0].t_proposition)); }
#line 3036 "pddl+.cpp"
    break;

  case 139: /* c_forall_effect: OPEN_BRAC c_forall OPEN_BRAC c_typed_var_list CLOSE_BRAC c_effect CLOSE_BRAC  */
#line 777 "pddl+.yacc"
     { (yyval.t_forall_effect)= new forall_effect((yyvsp[-1].t_effect_lists), (yyvsp[-3].t_var_symbol_list), current_analysis->var_tab_stack.pop());}
#line 3042 "pddl+.cpp"
    break;

  case 140: /* c_cond_effect: OPEN_BRAC WHEN c_goal_descriptor c_effects CLOSE_BRAC  */
#line 782 "pddl+.yacc"
     { (yyval.t_cond_effect)= new cond_effect((yyvsp[-2].t_goal),(yyvsp[-1].t_effect_lists)); }
#line 3048 "pddl+.cpp"
    break;

  case 141: /* c_assignment: OPEN_BRAC ASSIGN c_f_head c_f_exp CLOSE_BRAC  */
#line 787 "pddl+.yacc"
     { (yyval.t_assignment)= new assignment((yyvsp[-2].t_func_term),E_ASSIGN,(yyvsp[-1].t_expression)); }
#line 3054 "pddl+.cpp"
    break;

  case 142: /* c_assignment: OPEN_BRAC INCREASE c_f_head c_f_exp CLOSE_BRAC  */
#line 789 "pddl+.yacc"
     { (yyval.t_assignment)= new assignment((yyvsp[-2].t_func_term),E_INCREASE,(yyvsp[-1].t_expression)); }
#line 3060 "pddl+.cpp"
    break;

  case 143: /* c_assignment: OPEN_BRAC DECREASE c_f_head c_f_exp CLOSE_BRAC  */
#line 791 "pddl+.yacc"
     { (yyval.t_assignment)= new assignment((yyvsp[-2].t_func_term),E_DECREASE,(yyvsp[-1].t_expression)); }
#line 3066 "pddl+.cpp"
    break;

  case 144: /* c_assignment: OPEN_BRAC SCALE_UP c_f_head c_f_exp CLOSE_BRAC  */
#line 793 "pddl+.yacc"
     { (yyval.t_assignment)= new assignment((yyvsp[-2].t_func_term),E_SCALE_UP,(yyvsp[-1].t_expression)); }
#line 3072 "pddl+.cpp"
    break;

  case 145: /* c_assignment: OPEN_BRAC SCALE_DOWN c_f_head c_f_exp CLOSE_BRAC  */
#line 795 "pddl+.yacc"
     { (yyval.t_assignment)= new assignment((yyvsp[-2].t_func_term),E_SCALE_DOWN,(yyvsp[-1].t_expression)); }
#line 3078 "pddl+.cpp"
    break;

  case 146: /* c_f_exp: OPEN_BRAC HYPHEN c_f_exp CLOSE_BRAC  */
#line 800 "pddl+.yacc"
        { (yyval.t_expression)= new uminus_expression((yyvsp[-1].t_expression)); requires(E_FLUENTS); }
#line 3084 "pddl+.cpp"
    break;

  case 147: /* c_f_exp: OPEN_BRAC PLUS c_f_exp c_f_exp CLOSE_BRAC  */
#line 802 "pddl+.yacc"
        { (yyval.t_expression)= new plus_expression((yyvsp[-2].t_expression),(yyvsp[-1].t_expression)); requires(E_FLUENTS); }
#line 3090 "pddl+.cpp"
    break;

  case 148: /* c_f_exp: OPEN_BRAC HYPHEN c_f_exp c_f_exp CLOSE_BRAC  */
#line 804 "pddl+.yacc"
        { (yyval.t_expression)= new minus_expression((yyvsp[-2].t_expression),(yyvsp[-1].t_expression)); requires(E_FLUENTS); }
#line 3096 "pddl+.cpp"
    break;

  case 149: /* c_f_exp: OPEN_BRAC MUL c_f_exp c_f_exp CLOSE_BRAC  */
#line 806 "pddl+.yacc"
        { (yyval.t_expression)= new mul_expression((yyvsp[-2].t_expression),(yyvsp[-1].t_expression)); requires(E_FLUENTS); }
#line 3102 "pddl+.cpp"
    break;

  case 150: /* c_f_exp: OPEN_BRAC DIV c_f_exp c_f_exp CLOSE_BRAC  */
#line 808 "pddl+.yacc"
        { (yyval.t_expression)= new div_expression((yyvsp[-2].t_expression),(yyvsp[-1].t_expression)); requires(E_FLUENTS); }
#line 3108 "pddl+.cpp"
    break;

  case 151: /* c_f_exp: c_number  */
#line 809 "pddl+.yacc"
             { (yyval.t_expression)=(yyvsp[0].t_num_expression); }
#line 3114 "pddl+.cpp"
    break;

  case 152: /* c_f_exp: c_f_head  */
#line 810 "pddl+.yacc"
              { (yyval.t_expression)= (yyvsp[0].t_func_term); requires(E_FLUENTS); }
#line 3120 "pddl+.cpp"
    break;

  case 153: /* c_f_exp_t: OPEN_BRAC MUL HASHT c_f_exp CLOSE_BRAC  */
#line 815 "pddl+.yacc"
       { (yyval.t_expression)= new mul_expression(new special_val_expr(E_HASHT),(yyvsp[-1].t_expression)); }
#line 3126 "pddl+.cpp"
    break;

  case 154: /* c_f_exp_t: OPEN_BRAC MUL c_f_exp HASHT CLOSE_BRAC  */
#line 817 "pddl+.yacc"
       { (yyval.t_expression)= new mul_expression((yyvsp[-2].t_expression), new special_val_expr(E_HASHT)); }
#line 3132 "pddl+.cpp"
    break;

  case 155: /* c_f_exp_t: HASHT  */
#line 819 "pddl+.yacc"
       { (yyval.t_expression)= new special_val_expr(E_HASHT); }
#line 3138 "pddl+.cpp"
    break;

  case 156: /* c_number: INTVAL  */
#line 824 "pddl+.yacc"
              { (yyval.t_num_expression)=new int_expression((yyvsp[0].ival));   }
#line 3144 "pddl+.cpp"
    break;

  case 157: /* c_number: FLOATVAL  */
#line 825 "pddl+.yacc"
              { (yyval.t_num_expression)=new float_expression((yyvsp[0].fval)); }
#line 3150 "pddl+.cpp"
    break;

  case 158: /* c_f_head: OPEN_BRAC FUNCTION_SYMBOL c_parameter_symbols CLOSE_BRAC  */
#line 829 "pddl+.yacc"
        { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[-2].cp)), (yyvsp[-1].t_parameter_symbol_list)); delete [] (yyvsp[-2].cp); }
#line 3156 "pddl+.cpp"
    break;

  case 159: /* c_f_head: OPEN_BRAC NAME c_parameter_symbols CLOSE_BRAC  */
#line 832 "pddl+.yacc"
        { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[-2].cp)), (yyvsp[-1].t_parameter_symbol_list)); delete [] (yyvsp[-2].cp); }
#line 3162 "pddl+.cpp"
    break;

  case 160: /* c_f_head: FUNCTION_SYMBOL  */
#line 834 "pddl+.yacc"
        { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[0].cp)),
                            new parameter_symbol_list); delete [] (yyvsp[0].cp);}
#line 3169 "pddl+.cpp"
    break;

  case 161: /* c_ground_f_head: OPEN_BRAC FUNCTION_SYMBOL c_parameter_symbols CLOSE_BRAC  */
#line 852 "pddl+.yacc"
                { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[-2].cp)), (yyvsp[-1].t_parameter_symbol_list)); delete [] (yyvsp[-2].cp); }
#line 3175 "pddl+.cpp"
    break;

  case 162: /* c_ground_f_head: OPEN_BRAC NAME c_parameter_symbols CLOSE_BRAC  */
#line 854 "pddl+.yacc"
        { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[-2].cp)), (yyvsp[-1].t_parameter_symbol_list)); delete [] (yyvsp[-2].cp); }
#line 3181 "pddl+.cpp"
    break;

  case 163: /* c_ground_f_head: FUNCTION_SYMBOL  */
#line 856 "pddl+.yacc"
        { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[0].cp)),
                            new parameter_symbol_list); delete [] (yyvsp[0].cp);}
#line 3188 "pddl+.cpp"
    break;

  case 164: /* c_comparison_op: GREATER  */
#line 861 "pddl+.yacc"
               { (yyval.t_comparison_op)= E_GREATER; }
#line 3194 "pddl+.cpp"
    break;

  case 165: /* c_comparison_op: GREATEQ  */
#line 862 "pddl+.yacc"
               { (yyval.t_comparison_op)= E_GREATEQ; }
#line 3200 "pddl+.cpp"
    break;

  case 166: /* c_comparison_op: LESS  */
#line 863 "pddl+.yacc"
               { (yyval.t_comparison_op)= E_LESS; }
#line 3206 "pddl+.cpp"
    break;

  case 167: /* c_comparison_op: LESSEQ  */
#line 864 "pddl+.yacc"
               { (yyval.t_comparison_op)= E_LESSEQ; }
#line 3212 "pddl+.cpp"
    break;

  case 168: /* c_comparison_op: EQ  */
#line 865 "pddl+.yacc"
               { (yyval.t_comparison_op)= E_EQUALS; }
#line 3218 "pddl+.cpp"
    break;

  case 169: /* c_pre_goal_descriptor: c_pref_goal_descriptor  */
#line 878 "pddl+.yacc"
                {(yyval.t_goal)= (yyvsp[0].t_goal);}
#line 3224 "pddl+.cpp"
    break;

  case 170: /* c_pre_goal_descriptor: OPEN_BRAC AND c_pre_goal_descriptor_list CLOSE_BRAC  */
#line 880 "pddl+.yacc"
                {(yyval.t_goal) = new conj_goal((yyvsp[-1].t_goal_list));}
#line 3230 "pddl+.cpp"
    break;

  case 171: /* c_pre_goal_descriptor: OPEN_BRAC c_forall OPEN_BRAC c_typed_var_list CLOSE_BRAC c_pre_goal_descriptor CLOSE_BRAC  */
#line 883 "pddl+.yacc"
        {(yyval.t_goal)= new qfied_goal(E_FORALL,(yyvsp[-3].t_var_symbol_list),(yyvsp[-1].t_goal),current_analysis->var_tab_stack.pop());
        requires(E_UNIV_PRECS);}
#line 3237 "pddl+.cpp"
    break;

  case 172: /* c_pref_con_goal: OPEN_BRAC PREFERENCE c_constraint_goal CLOSE_BRAC  */
#line 889 "pddl+.yacc"
                {(yyval.t_con_goal) = new preference((yyvsp[-1].t_con_goal));requires(E_PREFERENCES);}
#line 3243 "pddl+.cpp"
    break;

  case 173: /* c_pref_con_goal: OPEN_BRAC PREFERENCE NAME c_constraint_goal CLOSE_BRAC  */
#line 891 "pddl+.yacc"
                {(yyval.t_con_goal) = new preference((yyvsp[-2].cp),(yyvsp[-1].t_con_goal));requires(E_PREFERENCES);}
#line 3249 "pddl+.cpp"
    break;

  case 174: /* c_pref_con_goal: OPEN_BRAC AND c_pref_con_goal_list CLOSE_BRAC  */
#line 893 "pddl+.yacc"
                {(yyval.t_con_goal) = new conj_goal((yyvsp[-1].t_goal_list));}
#line 3255 "pddl+.cpp"
    break;

  case 175: /* c_pref_con_goal: OPEN_BRAC c_forall OPEN_BRAC c_typed_var_list CLOSE_BRAC c_pref_goal CLOSE_BRAC  */
#line 896 "pddl+.yacc"
        {(yyval.t_con_goal)= new qfied_goal(E_FORALL,(yyvsp[-3].t_var_symbol_list),(yyvsp[-1].t_con_goal),current_analysis->var_tab_stack.pop());
                requires(E_UNIV_PRECS);}
#line 3262 "pddl+.cpp"
    break;

  case 176: /* c_pref_con_goal: c_constraint_goal  */
#line 899 "pddl+.yacc"
        {(yyval.t_con_goal) = (yyvsp[0].t_con_goal);}
#line 3268 "pddl+.cpp"
    break;

  case 177: /* c_pref_goal: OPEN_BRAC PREFERENCE c_constraint_goal CLOSE_BRAC  */
#line 904 "pddl+.yacc"
                {(yyval.t_con_goal) = new preference((yyvsp[-1].t_con_goal));requires(E_PREFERENCES);}
#line 3274 "pddl+.cpp"
    break;

  case 178: /* c_pref_goal: OPEN_BRAC PREFERENCE NAME c_constraint_goal CLOSE_BRAC  */
#line 906 "pddl+.yacc"
                {(yyval.t_con_goal) = new preference((yyvsp[-2].cp),(yyvsp[-1].t_con_goal));requires(E_PREFERENCES);}
#line 3280 "pddl+.cpp"
    break;

  case 179: /* c_pref_goal: OPEN_BRAC AND c_pref_con_goal_list CLOSE_BRAC  */
#line 908 "pddl+.yacc"
                {(yyval.t_con_goal) = new conj_goal((yyvsp[-1].t_goal_list));}
#line 3286 "pddl+.cpp"
    break;

  case 180: /* c_pref_goal: OPEN_BRAC c_forall OPEN_BRAC c_typed_var_list CLOSE_BRAC c_pref_goal CLOSE_BRAC  */
#line 911 "pddl+.yacc"
        {(yyval.t_con_goal)= new qfied_goal(E_FORALL,(yyvsp[-3].t_var_symbol_list),(yyvsp[-1].t_con_goal),current_analysis->var_tab_stack.pop());
                requires(E_UNIV_PRECS);}
#line 3293 "pddl+.cpp"
    break;

  case 181: /* c_pref_con_goal_list: c_pref_con_goal_list c_pref_con_goal  */
#line 917 "pddl+.yacc"
                {(yyval.t_goal_list)=(yyvsp[-1].t_goal_list); (yyvsp[-1].t_goal_list)->push_back((yyvsp[0].t_con_goal));}
#line 3299 "pddl+.cpp"
    break;

  case 182: /* c_pref_con_goal_list: %empty  */
#line 919 "pddl+.yacc"
                {(yyval.t_goal_list)= new goal_list;}
#line 3305 "pddl+.cpp"
    break;

  case 183: /* c_pref_goal_descriptor: OPEN_BRAC PREFERENCE c_goal_descriptor CLOSE_BRAC  */
#line 924 "pddl+.yacc"
        {(yyval.t_goal)= new preference((yyvsp[-1].t_goal)); requires(E_PREFERENCES);}
#line 3311 "pddl+.cpp"
    break;

  case 184: /* c_pref_goal_descriptor: OPEN_BRAC PREFERENCE NAME c_goal_descriptor CLOSE_BRAC  */
#line 926 "pddl+.yacc"
        {(yyval.t_goal)= new preference((yyvsp[-2].cp),(yyvsp[-1].t_goal)); requires(E_PREFERENCES);}
#line 3317 "pddl+.cpp"
    break;

  case 185: /* c_pref_goal_descriptor: c_goal_descriptor  */
#line 928 "pddl+.yacc"
        {(yyval.t_goal)=(yyvsp[0].t_goal);}
#line 3323 "pddl+.cpp"
    break;

  case 186: /* c_constraint_goal_list: c_constraint_goal_list c_constraint_goal  */
#line 933 "pddl+.yacc"
        {(yyval.t_goal_list) = (yyvsp[-1].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[0].t_con_goal));}
#line 3329 "pddl+.cpp"
    break;

  case 187: /* c_constraint_goal_list: %empty  */
#line 935 "pddl+.yacc"
        {(yyval.t_goal_list) = new goal_list;}
#line 3335 "pddl+.cpp"
    break;

  case 188: /* c_constraint_goal: OPEN_BRAC AND c_constraint_goal_list CLOSE_BRAC  */
#line 940 "pddl+.yacc"
                {(yyval.t_con_goal)= new conj_goal((yyvsp[-1].t_goal_list));}
#line 3341 "pddl+.cpp"
    break;

  case 189: /* c_constraint_goal: OPEN_BRAC c_forall OPEN_BRAC c_typed_var_list CLOSE_BRAC c_constraint_goal CLOSE_BRAC  */
#line 942 "pddl+.yacc"
                {(yyval.t_con_goal) = new qfied_goal(E_FORALL,(yyvsp[-3].t_var_symbol_list),(yyvsp[-1].t_con_goal),current_analysis->var_tab_stack.pop());
        requires(E_UNIV_PRECS);}
#line 3348 "pddl+.cpp"
    break;

  case 190: /* c_constraint_goal: OPEN_BRAC AT_END c_goal_descriptor CLOSE_BRAC  */
#line 945 "pddl+.yacc"
                {(yyval.t_con_goal) = new constraint_goal(E_ATEND,(yyvsp[-1].t_goal));}
#line 3354 "pddl+.cpp"
    break;

  case 191: /* c_constraint_goal: OPEN_BRAC ALWAYS c_goal_descriptor CLOSE_BRAC  */
#line 947 "pddl+.yacc"
                {(yyval.t_con_goal) = new constraint_goal(E_ALWAYS,(yyvsp[-1].t_goal));}
#line 3360 "pddl+.cpp"
    break;

  case 192: /* c_constraint_goal: OPEN_BRAC SOMETIME c_goal_descriptor CLOSE_BRAC  */
#line 949 "pddl+.yacc"
                {(yyval.t_con_goal) = new constraint_goal(E_SOMETIME,(yyvsp[-1].t_goal));}
#line 3366 "pddl+.cpp"
    break;

  case 193: /* c_constraint_goal: OPEN_BRAC WITHIN c_number c_goal_descriptor CLOSE_BRAC  */
#line 951 "pddl+.yacc"
                {(yyval.t_con_goal) = new constraint_goal(E_WITHIN,(yyvsp[-1].t_goal),NULL,(yyvsp[-2].t_num_expression)->double_value(),0.0);delete (yyvsp[-2].t_num_expression);}
#line 3372 "pddl+.cpp"
    break;

  case 194: /* c_constraint_goal: OPEN_BRAC ATMOSTONCE c_goal_descriptor CLOSE_BRAC  */
#line 953 "pddl+.yacc"
                {(yyval.t_con_goal) = new constraint_goal(E_ATMOSTONCE,(yyvsp[-1].t_goal));}
#line 3378 "pddl+.cpp"
    break;

  case 195: /* c_constraint_goal: OPEN_BRAC SOMETIMEAFTER c_goal_descriptor c_goal_descriptor CLOSE_BRAC  */
#line 955 "pddl+.yacc"
                {(yyval.t_con_goal) = new constraint_goal(E_SOMETIMEAFTER,(yyvsp[-1].t_goal),(yyvsp[-2].t_goal));}
#line 3384 "pddl+.cpp"
    break;

  case 196: /* c_constraint_goal: OPEN_BRAC SOMETIMEBEFORE c_goal_descriptor c_goal_descriptor CLOSE_BRAC  */
#line 957 "pddl+.yacc"
                {(yyval.t_con_goal) = new constraint_goal(E_SOMETIMEBEFORE,(yyvsp[-1].t_goal),(yyvsp[-2].t_goal));}
#line 3390 "pddl+.cpp"
    break;

  case 197: /* c_constraint_goal: OPEN_BRAC ALWAYSWITHIN c_number c_goal_descriptor c_goal_descriptor CLOSE_BRAC  */
#line 959 "pddl+.yacc"
                {(yyval.t_con_goal) = new constraint_goal(E_ALWAYSWITHIN,(yyvsp[-1].t_goal),(yyvsp[-2].t_goal),(yyvsp[-3].t_num_expression)->double_value(),0.0);delete (yyvsp[-3].t_num_expression);}
#line 3396 "pddl+.cpp"
    break;

  case 198: /* c_constraint_goal: OPEN_BRAC HOLDDURING c_number c_number c_goal_descriptor CLOSE_BRAC  */
#line 961 "pddl+.yacc"
                {(yyval.t_con_goal) = new constraint_goal(E_HOLDDURING,(yyvsp[-1].t_goal),NULL,(yyvsp[-2].t_num_expression)->double_value(),(yyvsp[-3].t_num_expression)->double_value());delete (yyvsp[-3].t_num_expression);delete (yyvsp[-2].t_num_expression);}
#line 3402 "pddl+.cpp"
    break;

  case 199: /* c_constraint_goal: OPEN_BRAC HOLDAFTER c_number c_goal_descriptor CLOSE_BRAC  */
#line 963 "pddl+.yacc"
                {(yyval.t_con_goal) = new constraint_goal(E_HOLDAFTER,(yyvsp[-1].t_goal),NULL,0.0,(yyvsp[-2].t_num_expression)->double_value());delete (yyvsp[-2].t_num_expression);}
#line 3408 "pddl+.cpp"
    break;

  case 200: /* c_goal_descriptor: c_proposition  */
#line 968 "pddl+.yacc"
       {(yyval.t_goal)= new simple_goal((yyvsp[0].t_proposition),E_POS);}
#line 3414 "pddl+.cpp"
    break;

  case 201: /* c_goal_descriptor: OPEN_BRAC NOT c_goal_descriptor CLOSE_BRAC  */
#line 970 "pddl+.yacc"
       {(yyval.t_goal)= new neg_goal((yyvsp[-1].t_goal));simple_goal * s = dynamic_cast<simple_goal *>((yyvsp[-1].t_goal));
       if(s && s->getProp()->head->getName()=="=") {requires(E_EQUALITY);} 
       else{requires(E_NEGATIVE_PRECONDITIONS);};}
#line 3422 "pddl+.cpp"
    break;

  case 202: /* c_goal_descriptor: OPEN_BRAC AND c_goal_list CLOSE_BRAC  */
#line 974 "pddl+.yacc"
       {(yyval.t_goal)= new conj_goal((yyvsp[-1].t_goal_list));}
#line 3428 "pddl+.cpp"
    break;

  case 203: /* c_goal_descriptor: OPEN_BRAC OR c_goal_list CLOSE_BRAC  */
#line 976 "pddl+.yacc"
       {(yyval.t_goal)= new disj_goal((yyvsp[-1].t_goal_list));
        requires(E_DISJUNCTIVE_PRECONDS);}
#line 3435 "pddl+.cpp"
    break;

  case 204: /* c_goal_descriptor: OPEN_BRAC IMPLY c_goal_descriptor c_goal_descriptor CLOSE_BRAC  */
#line 979 "pddl+.yacc"
       {(yyval.t_goal)= new imply_goal((yyvsp[-2].t_goal),(yyvsp[-1].t_goal));
        requires(E_DISJUNCTIVE_PRECONDS);}
#line 3442 "pddl+.cpp"
    break;

  case 205: /* c_goal_descriptor: OPEN_BRAC c_quantifier OPEN_BRAC c_typed_var_list CLOSE_BRAC c_goal_descriptor CLOSE_BRAC  */
#line 983 "pddl+.yacc"
       {(yyval.t_goal)= new qfied_goal((yyvsp[-5].t_quantifier),(yyvsp[-3].t_var_symbol_list),(yyvsp[-1].t_goal),current_analysis->var_tab_stack.pop());}
#line 3448 "pddl+.cpp"
    break;

  case 206: /* c_goal_descriptor: OPEN_BRAC c_comparison_op c_f_exp c_f_exp CLOSE_BRAC  */
#line 985 "pddl+.yacc"
       {(yyval.t_goal)= new comparison((yyvsp[-3].t_comparison_op),(yyvsp[-2].t_expression),(yyvsp[-1].t_expression)); 
        requires(E_FLUENTS);}
#line 3455 "pddl+.cpp"
    break;

  case 207: /* c_pre_goal_descriptor_list: c_pre_goal_descriptor_list c_pre_goal_descriptor  */
#line 991 "pddl+.yacc"
                {(yyval.t_goal_list)=(yyvsp[-1].t_goal_list); (yyvsp[-1].t_goal_list)->push_back((yyvsp[0].t_goal));}
#line 3461 "pddl+.cpp"
    break;

  case 208: /* c_pre_goal_descriptor_list: %empty  */
#line 993 "pddl+.yacc"
                {(yyval.t_goal_list)= new goal_list;}
#line 3467 "pddl+.cpp"
    break;

  case 209: /* c_goal_list: c_goal_list c_goal_descriptor  */
#line 998 "pddl+.yacc"
        {(yyval.t_goal_list)=(yyvsp[-1].t_goal_list); (yyvsp[-1].t_goal_list)->push_back((yyvsp[0].t_goal));}
#line 3473 "pddl+.cpp"
    break;

  case 210: /* c_goal_list: %empty  */
#line 1000 "pddl+.yacc"
        {(yyval.t_goal_list)= new goal_list;}
#line 3479 "pddl+.cpp"
    break;

  case 211: /* c_quantifier: c_forall  */
#line 1004 "pddl+.yacc"
             {(yyval.t_quantifier)=(yyvsp[0].t_quantifier);}
#line 3485 "pddl+.cpp"
    break;

  case 212: /* c_quantifier: c_exists  */
#line 1005 "pddl+.yacc"
             {(yyval.t_quantifier)=(yyvsp[0].t_quantifier);}
#line 3491 "pddl+.cpp"
    break;

  case 213: /* c_forall: FORALL  */
#line 1010 "pddl+.yacc"
       {(yyval.t_quantifier)=E_FORALL; 
        current_analysis->var_tab_stack.push(
        		current_analysis->buildForallTab());}
#line 3499 "pddl+.cpp"
    break;

  case 214: /* c_exists: EXISTS  */
#line 1017 "pddl+.yacc"
       {(yyval.t_quantifier)=E_EXISTS;
        current_analysis->var_tab_stack.push(
        	current_analysis->buildExistsTab());}
#line 3507 "pddl+.cpp"
    break;

  case 215: /* c_proposition: OPEN_BRAC c_pred_symbol c_parameter_symbols CLOSE_BRAC  */
#line 1024 "pddl+.yacc"
        {(yyval.t_proposition)=new proposition((yyvsp[-2].t_pred_symbol),(yyvsp[-1].t_parameter_symbol_list));}
#line 3513 "pddl+.cpp"
    break;

  case 216: /* c_derived_proposition: OPEN_BRAC c_pred_symbol c_typed_var_list CLOSE_BRAC  */
#line 1029 "pddl+.yacc"
         {(yyval.t_proposition) = new proposition((yyvsp[-2].t_pred_symbol),(yyvsp[-1].t_var_symbol_list));}
#line 3519 "pddl+.cpp"
    break;

  case 217: /* c_init_proposition: OPEN_BRAC c_init_pred_symbol c_parameter_symbols CLOSE_BRAC  */
#line 1034 "pddl+.yacc"
        {(yyval.t_proposition)=new proposition((yyvsp[-2].t_pred_symbol),(yyvsp[-1].t_parameter_symbol_list));}
#line 3525 "pddl+.cpp"
    break;

  case 218: /* c_predicates: OPEN_BRAC PREDS c_pred_decls CLOSE_BRAC  */
#line 1039 "pddl+.yacc"
        {(yyval.t_pred_decl_list)= (yyvsp[-1].t_pred_decl_list);}
#line 3531 "pddl+.cpp"
    break;

  case 219: /* c_predicates: OPEN_BRAC PREDS error CLOSE_BRAC  */
#line 1041 "pddl+.yacc"
        {yyerrok; (yyval.t_pred_decl_list)=NULL;
	 log_error(E_FATAL,"Syntax error in (:predicates ...)");
	}
#line 3539 "pddl+.cpp"
    break;

  case 220: /* c_functions_def: OPEN_BRAC FUNCTIONS c_func_decls CLOSE_BRAC  */
#line 1048 "pddl+.yacc"
        {(yyval.t_func_decl_list)= (yyvsp[-1].t_func_decl_list);}
#line 3545 "pddl+.cpp"
    break;

  case 221: /* c_functions_def: OPEN_BRAC FUNCTIONS error CLOSE_BRAC  */
#line 1050 "pddl+.yacc"
        {yyerrok; (yyval.t_func_decl_list)=NULL;
	 log_error(E_FATAL,"Syntax error in (:functions ...)");
	}
#line 3553 "pddl+.cpp"
    break;

  case 222: /* c_constraints_def: OPEN_BRAC CONSTRAINTS c_constraint_goal CLOSE_BRAC  */
#line 1057 "pddl+.yacc"
                {(yyval.t_con_goal) = (yyvsp[-1].t_con_goal);}
#line 3559 "pddl+.cpp"
    break;

  case 223: /* c_constraints_def: OPEN_BRAC CONSTRAINTS error CLOSE_BRAC  */
#line 1059 "pddl+.yacc"
    {yyerrok; (yyval.t_con_goal)=NULL;
      log_error(E_FATAL,"Syntax error in (:constraints ...)");
      }
#line 3567 "pddl+.cpp"
    break;

  case 224: /* c_constraints_probdef: OPEN_BRAC CONSTRAINTS c_pref_con_goal CLOSE_BRAC  */
#line 1066 "pddl+.yacc"
                {(yyval.t_con_goal) = (yyvsp[-1].t_con_goal);}
#line 3573 "pddl+.cpp"
    break;

  case 225: /* c_constraints_probdef: OPEN_BRAC CONSTRAINTS error CLOSE_BRAC  */
#line 1068 "pddl+.yacc"
         {yyerrok; (yyval.t_con_goal)=NULL;
      log_error(E_FATAL,"Syntax error in (:constraints ...)");
      }
#line 3581 "pddl+.cpp"
    break;

  case 226: /* c_structure_defs: c_structure_defs c_structure_def  */
#line 1074 "pddl+.yacc"
                                     { (yyval.t_structure_store)=(yyvsp[-1].t_structure_store); (yyval.t_structure_store)->push_back((yyvsp[0].t_structure_def)); }
#line 3587 "pddl+.cpp"
    break;

  case 227: /* c_structure_defs: c_structure_def  */
#line 1075 "pddl+.yacc"
                     { (yyval.t_structure_store)= new structure_store; (yyval.t_structure_store)->push_back((yyvsp[0].t_structure_def)); }
#line 3593 "pddl+.cpp"
    break;

  case 228: /* c_structure_def: c_action_def  */
#line 1079 "pddl+.yacc"
                          { (yyval.t_structure_def)= (yyvsp[0].t_action_def); }
#line 3599 "pddl+.cpp"
    break;

  case 229: /* c_structure_def: c_event_def  */
#line 1080 "pddl+.yacc"
                          { (yyval.t_structure_def)= (yyvsp[0].t_event_def); requires(E_TIME); }
#line 3605 "pddl+.cpp"
    break;

  case 230: /* c_structure_def: c_process_def  */
#line 1081 "pddl+.yacc"
                          { (yyval.t_structure_def)= (yyvsp[0].t_process_def); requires(E_TIME); }
#line 3611 "pddl+.cpp"
    break;

  case 231: /* c_structure_def: c_durative_action_def  */
#line 1082 "pddl+.yacc"
                          { (yyval.t_structure_def)= (yyvsp[0].t_durative_action_def); requires(E_DURATIVE_ACTIONS); }
#line 3617 "pddl+.cpp"
    break;

  case 232: /* c_structure_def: c_derivation_rule  */
#line 1083 "pddl+.yacc"
                          { (yyval.t_structure_def)= (yyvsp[0].t_derivation_rule); requires(E_DERIVED_PREDICATES);}
#line 3623 "pddl+.cpp"
    break;

  case 233: /* c_rule_head: DERIVED  */
#line 1087 "pddl+.yacc"
            {(yyval.t_dummy)= 0; 
    	current_analysis->var_tab_stack.push(
    					current_analysis->buildRuleTab());}
#line 3631 "pddl+.cpp"
    break;

  case 234: /* c_derivation_rule: OPEN_BRAC c_rule_head c_derived_proposition c_goal_descriptor CLOSE_BRAC  */
#line 1098 "pddl+.yacc"
        {(yyval.t_derivation_rule) = new derivation_rule((yyvsp[-2].t_proposition),(yyvsp[-1].t_goal),current_analysis->var_tab_stack.pop());}
#line 3637 "pddl+.cpp"
    break;

  case 235: /* c_action_def: OPEN_BRAC ACTION NAME c_args_head OPEN_BRAC c_typed_var_list CLOSE_BRAC PRE c_pre_goal_descriptor EFFECTS c_effect CLOSE_BRAC  */
#line 1110 "pddl+.yacc"
    { (yyval.t_action_def)= current_analysis->buildAction(current_analysis->op_tab.symbol_put((yyvsp[-9].cp)),
			(yyvsp[-6].t_var_symbol_list),(yyvsp[-3].t_goal),(yyvsp[-1].t_effect_lists),
			current_analysis->var_tab_stack.pop()); delete [] (yyvsp[-9].cp); }
#line 3645 "pddl+.cpp"
    break;

  case 236: /* c_action_def: OPEN_BRAC ACTION error CLOSE_BRAC  */
#line 1114 "pddl+.yacc"
        {yyerrok; 
	 log_error(E_FATAL,"Syntax error in action declaration.");
	 (yyval.t_action_def)= NULL; }
#line 3653 "pddl+.cpp"
    break;

  case 237: /* c_event_def: OPEN_BRAC EVENT NAME c_args_head OPEN_BRAC c_typed_var_list CLOSE_BRAC PRE c_goal_descriptor EFFECTS c_effect CLOSE_BRAC  */
#line 1127 "pddl+.yacc"
    {(yyval.t_event_def)= current_analysis->buildEvent(current_analysis->op_tab.symbol_put((yyvsp[-9].cp)),
		   (yyvsp[-6].t_var_symbol_list),(yyvsp[-3].t_goal),(yyvsp[-1].t_effect_lists),
		   current_analysis->var_tab_stack.pop()); delete [] (yyvsp[-9].cp);}
#line 3661 "pddl+.cpp"
    break;

  case 238: /* c_event_def: OPEN_BRAC EVENT error CLOSE_BRAC  */
#line 1132 "pddl+.yacc"
        {yyerrok; 
	 log_error(E_FATAL,"Syntax error in event declaration.");
	 (yyval.t_event_def)= NULL; }
#line 3669 "pddl+.cpp"
    break;

  case 239: /* c_process_def: OPEN_BRAC PROCESS NAME c_args_head OPEN_BRAC c_typed_var_list CLOSE_BRAC PRE c_goal_descriptor EFFECTS c_proc_effect CLOSE_BRAC  */
#line 1144 "pddl+.yacc"
    {(yyval.t_process_def)= current_analysis->buildProcess(current_analysis->op_tab.symbol_put((yyvsp[-9].cp)),
		     (yyvsp[-6].t_var_symbol_list),(yyvsp[-3].t_goal),(yyvsp[-1].t_effect_lists),
                     current_analysis->var_tab_stack.pop()); delete [] (yyvsp[-9].cp);}
#line 3677 "pddl+.cpp"
    break;

  case 240: /* c_process_def: OPEN_BRAC PROCESS error CLOSE_BRAC  */
#line 1148 "pddl+.yacc"
        {yyerrok; 
	 log_error(E_FATAL,"Syntax error in process declaration.");
	 (yyval.t_process_def)= NULL; }
#line 3685 "pddl+.cpp"
    break;

  case 241: /* c_durative_action_def: OPEN_BRAC DURATIVE_ACTION NAME c_args_head OPEN_BRAC c_typed_var_list CLOSE_BRAC DURATION c_duration_constraint c_da_def_body CLOSE_BRAC  */
#line 1160 "pddl+.yacc"
    { (yyval.t_durative_action_def)= (yyvsp[-1].t_durative_action_def);
      (yyval.t_durative_action_def)->name= current_analysis->op_tab.symbol_put((yyvsp[-8].cp));
      (yyval.t_durative_action_def)->symtab= current_analysis->var_tab_stack.pop();
      (yyval.t_durative_action_def)->parameters= (yyvsp[-5].t_var_symbol_list);
      (yyval.t_durative_action_def)->dur_constraint= (yyvsp[-2].t_goal); 
      delete [] (yyvsp[-8].cp);
    }
#line 3697 "pddl+.cpp"
    break;

  case 242: /* c_durative_action_def: OPEN_BRAC DURATIVE_ACTION error CLOSE_BRAC  */
#line 1169 "pddl+.yacc"
        {yyerrok; 
	 log_error(E_FATAL,"Syntax error in durative-action declaration.");
	 (yyval.t_durative_action_def)= NULL; }
#line 3705 "pddl+.cpp"
    break;

  case 243: /* c_da_def_body: c_da_def_body EFFECTS c_da_effect  */
#line 1176 "pddl+.yacc"
        {(yyval.t_durative_action_def)=(yyvsp[-2].t_durative_action_def); (yyval.t_durative_action_def)->effects=(yyvsp[0].t_effect_lists);}
#line 3711 "pddl+.cpp"
    break;

  case 244: /* c_da_def_body: c_da_def_body CONDITION c_da_gd  */
#line 1178 "pddl+.yacc"
        {(yyval.t_durative_action_def)=(yyvsp[-2].t_durative_action_def); (yyval.t_durative_action_def)->precondition=(yyvsp[0].t_goal);}
#line 3717 "pddl+.cpp"
    break;

  case 245: /* c_da_def_body: %empty  */
#line 1179 "pddl+.yacc"
                 {(yyval.t_durative_action_def)= current_analysis->buildDurativeAction();}
#line 3723 "pddl+.cpp"
    break;

  case 246: /* c_da_gd: c_timed_gd  */
#line 1184 "pddl+.yacc"
       { (yyval.t_goal)=(yyvsp[0].t_goal); }
#line 3729 "pddl+.cpp"
    break;

  case 247: /* c_da_gd: OPEN_BRAC AND c_da_gds CLOSE_BRAC  */
#line 1186 "pddl+.yacc"
       { (yyval.t_goal)= new conj_goal((yyvsp[-1].t_goal_list)); }
#line 3735 "pddl+.cpp"
    break;

  case 248: /* c_da_gds: c_da_gds c_da_gd  */
#line 1191 "pddl+.yacc"
       { (yyval.t_goal_list)=(yyvsp[-1].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[0].t_goal)); }
#line 3741 "pddl+.cpp"
    break;

  case 249: /* c_da_gds: %empty  */
#line 1193 "pddl+.yacc"
       { (yyval.t_goal_list)= new goal_list; }
#line 3747 "pddl+.cpp"
    break;

  case 250: /* c_timed_gd: OPEN_BRAC AT_START c_goal_descriptor CLOSE_BRAC  */
#line 1198 "pddl+.yacc"
        {(yyval.t_goal)= new timed_goal((yyvsp[-1].t_goal),E_AT_START);}
#line 3753 "pddl+.cpp"
    break;

  case 251: /* c_timed_gd: OPEN_BRAC AT_END c_goal_descriptor CLOSE_BRAC  */
#line 1200 "pddl+.yacc"
        {(yyval.t_goal)= new timed_goal((yyvsp[-1].t_goal),E_AT_END);}
#line 3759 "pddl+.cpp"
    break;

  case 252: /* c_timed_gd: OPEN_BRAC OVER_ALL c_goal_descriptor CLOSE_BRAC  */
#line 1202 "pddl+.yacc"
        {(yyval.t_goal)= new timed_goal((yyvsp[-1].t_goal),E_OVER_ALL);}
#line 3765 "pddl+.cpp"
    break;

  case 253: /* c_timed_gd: OPEN_BRAC PREFERENCE NAME c_timed_gd CLOSE_BRAC  */
#line 1204 "pddl+.yacc"
                {timed_goal * tg = dynamic_cast<timed_goal *>((yyvsp[-1].t_goal));
		(yyval.t_goal) = new timed_goal(new preference((yyvsp[-2].cp),tg->clearGoal()),tg->getTime());
			delete tg;
			requires(E_PREFERENCES);}
#line 3774 "pddl+.cpp"
    break;

  case 254: /* c_timed_gd: OPEN_BRAC PREFERENCE c_timed_gd CLOSE_BRAC  */
#line 1209 "pddl+.yacc"
        {(yyval.t_goal) = new preference((yyvsp[-1].t_goal));requires(E_PREFERENCES);}
#line 3780 "pddl+.cpp"
    break;

  case 255: /* c_args_head: ARGS  */
#line 1213 "pddl+.yacc"
         {(yyval.t_dummy)= 0; current_analysis->var_tab_stack.push(
    				current_analysis->buildOpTab());}
#line 3787 "pddl+.cpp"
    break;

  case 256: /* c_require_key: EQUALITY  */
#line 1218 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_EQUALITY;}
#line 3793 "pddl+.cpp"
    break;

  case 257: /* c_require_key: STRIPS  */
#line 1219 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_STRIPS;}
#line 3799 "pddl+.cpp"
    break;

  case 258: /* c_require_key: TYPING  */
#line 1221 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_TYPING;}
#line 3805 "pddl+.cpp"
    break;

  case 259: /* c_require_key: NEGATIVE_PRECONDITIONS  */
#line 1223 "pddl+.yacc"
                                 {(yyval.t_pddl_req_flag)= E_NEGATIVE_PRECONDITIONS;}
#line 3811 "pddl+.cpp"
    break;

  case 260: /* c_require_key: DISJUNCTIVE_PRECONDS  */
#line 1225 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_DISJUNCTIVE_PRECONDS;}
#line 3817 "pddl+.cpp"
    break;

  case 261: /* c_require_key: EXT_PRECS  */
#line 1226 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_EXT_PRECS;}
#line 3823 "pddl+.cpp"
    break;

  case 262: /* c_require_key: UNIV_PRECS  */
#line 1227 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_UNIV_PRECS;}
#line 3829 "pddl+.cpp"
    break;

  case 263: /* c_require_key: COND_EFFS  */
#line 1228 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_COND_EFFS;}
#line 3835 "pddl+.cpp"
    break;

  case 264: /* c_require_key: FLUENTS  */
#line 1229 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_FLUENTS;}
#line 3841 "pddl+.cpp"
    break;

  case 265: /* c_require_key: DURATIVE_ACTIONS  */
#line 1231 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_DURATIVE_ACTIONS;}
#line 3847 "pddl+.cpp"
    break;

  case 266: /* c_require_key: TIME  */
#line 1232 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_TIME |
                      E_FLUENTS |
                      E_DURATIVE_ACTIONS; }
#line 3855 "pddl+.cpp"
    break;

  case 267: /* c_require_key: ADL  */
#line 1236 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_STRIPS |
		      E_TYPING | 
		      E_NEGATIVE_PRECONDITIONS |
		      E_DISJUNCTIVE_PRECONDS |
		      E_EQUALITY |
		      E_EXT_PRECS |
		      E_UNIV_PRECS |
		      E_COND_EFFS;}
#line 3868 "pddl+.cpp"
    break;

  case 268: /* c_require_key: QUANT_PRECS  */
#line 1245 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_EXT_PRECS |
		      E_UNIV_PRECS;}
#line 3875 "pddl+.cpp"
    break;

  case 269: /* c_require_key: DURATION_INEQUALITIES  */
#line 1249 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_DURATION_INEQUALITIES;}
#line 3881 "pddl+.cpp"
    break;

  case 270: /* c_require_key: CONTINUOUS_EFFECTS  */
#line 1252 "pddl+.yacc"
                 {(yyval.t_pddl_req_flag)= E_CONTINUOUS_EFFECTS;}
#line 3887 "pddl+.cpp"
    break;

  case 271: /* c_require_key: DERIVED_PREDICATES  */
#line 1254 "pddl+.yacc"
                                 {(yyval.t_pddl_req_flag) = E_DERIVED_PREDICATES;}
#line 3893 "pddl+.cpp"
    break;

  case 272: /* c_require_key: TIMED_INITIAL_LITERALS  */
#line 1256 "pddl+.yacc"
                                {(yyval.t_pddl_req_flag) = E_TIMED_INITIAL_LITERALS;}
#line 3899 "pddl+.cpp"
    break;

  case 273: /* c_require_key: PREFERENCES  */
#line 1258 "pddl+.yacc"
                                {(yyval.t_pddl_req_flag) = E_PREFERENCES;}
#line 3905 "pddl+.cpp"
    break;

  case 274: /* c_require_key: CONSTRAINTS  */
#line 1260 "pddl+.yacc"
                {(yyval.t_pddl_req_flag) = E_CONSTRAINTS;}
#line 3911 "pddl+.cpp"
    break;

  case 275: /* c_require_key: NAME  */
#line 1262 "pddl+.yacc"
      {log_error(E_WARNING,"Unrecognised requirements declaration ");
       (yyval.t_pddl_req_flag)= 0; delete [] (yyvsp[0].cp);}
#line 3918 "pddl+.cpp"
    break;

  case 276: /* c_domain_constants: OPEN_BRAC CONSTANTS c_typed_consts CLOSE_BRAC  */
#line 1268 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[-1].t_const_symbol_list);}
#line 3924 "pddl+.cpp"
    break;

  case 277: /* c_type_names: OPEN_BRAC TYPES c_typed_types CLOSE_BRAC  */
#line 1272 "pddl+.yacc"
    {(yyval.t_type_list)=(yyvsp[-1].t_type_list); requires(E_TYPING);}
#line 3930 "pddl+.cpp"
    break;

  case 278: /* c_problem: OPEN_BRAC DEFINE OPEN_BRAC PROBLEM NAME CLOSE_BRAC OPEN_BRAC FORDOMAIN NAME CLOSE_BRAC c_problem_body CLOSE_BRAC  */
#line 1282 "pddl+.yacc"
            {(yyval.t_problem)=(yyvsp[-1].t_problem); (yyval.t_problem)->name = (yyvsp[-7].cp); (yyval.t_problem)->domain_name = (yyvsp[-3].cp);}
#line 3936 "pddl+.cpp"
    break;

  case 279: /* c_problem: OPEN_BRAC DEFINE OPEN_BRAC PROBLEM error  */
#line 1284 "pddl+.yacc"
        {yyerrok; (yyval.t_problem)=NULL;
       	log_error(E_FATAL,"Syntax error in problem definition."); }
#line 3943 "pddl+.cpp"
    break;

  case 280: /* c_problem_body: c_domain_require_def c_problem_body  */
#line 1290 "pddl+.yacc"
                                         {(yyval.t_problem)=(yyvsp[0].t_problem); (yyval.t_problem)->req= (yyvsp[-1].t_pddl_req_flag);}
#line 3949 "pddl+.cpp"
    break;

  case 281: /* c_problem_body: c_objects c_problem_body  */
#line 1291 "pddl+.yacc"
                                    {(yyval.t_problem)=(yyvsp[0].t_problem); (yyval.t_problem)->objects= (yyvsp[-1].t_const_symbol_list);}
#line 3955 "pddl+.cpp"
    break;

  case 282: /* c_problem_body: c_initial_state c_problem_body  */
#line 1292 "pddl+.yacc"
                                    {(yyval.t_problem)=(yyvsp[0].t_problem); (yyval.t_problem)->initial_state= (yyvsp[-1].t_effect_lists);}
#line 3961 "pddl+.cpp"
    break;

  case 283: /* c_problem_body: c_goal_spec c_problem_body  */
#line 1293 "pddl+.yacc"
                                    {(yyval.t_problem)=(yyvsp[0].t_problem); (yyval.t_problem)->the_goal= (yyvsp[-1].t_goal);}
#line 3967 "pddl+.cpp"
    break;

  case 284: /* c_problem_body: c_constraints_probdef c_problem_body  */
#line 1295 "pddl+.yacc"
                                                                        {(yyval.t_problem)=(yyvsp[0].t_problem); (yyval.t_problem)->constraints = (yyvsp[-1].t_con_goal);}
#line 3973 "pddl+.cpp"
    break;

  case 285: /* c_problem_body: c_metric_spec c_problem_body  */
#line 1296 "pddl+.yacc"
                                    {(yyval.t_problem)=(yyvsp[0].t_problem); (yyval.t_problem)->metric= (yyvsp[-1].t_metric);}
#line 3979 "pddl+.cpp"
    break;

  case 286: /* c_problem_body: c_length_spec c_problem_body  */
#line 1297 "pddl+.yacc"
                                    {(yyval.t_problem)=(yyvsp[0].t_problem); (yyval.t_problem)->length= (yyvsp[-1].t_length_spec);}
#line 3985 "pddl+.cpp"
    break;

  case 287: /* c_problem_body: %empty  */
#line 1298 "pddl+.yacc"
                                    {(yyval.t_problem)=new problem;}
#line 3991 "pddl+.cpp"
    break;

  case 288: /* c_objects: OPEN_BRAC OBJECTS c_typed_consts CLOSE_BRAC  */
#line 1301 "pddl+.yacc"
                                                        {(yyval.t_const_symbol_list)=(yyvsp[-1].t_const_symbol_list);}
#line 3997 "pddl+.cpp"
    break;

  case 289: /* c_initial_state: OPEN_BRAC INITIALLY c_init_els CLOSE_BRAC  */
#line 1304 "pddl+.yacc"
                                                            {(yyval.t_effect_lists)=(yyvsp[-1].t_effect_lists);}
#line 4003 "pddl+.cpp"
    break;

  case 290: /* c_goals: GOALS  */
#line 1307 "pddl+.yacc"
                {(yyval.vtab) = current_analysis->buildOpTab();}
#line 4009 "pddl+.cpp"
    break;

  case 291: /* c_goal_spec: OPEN_BRAC c_goals c_pre_goal_descriptor CLOSE_BRAC  */
#line 1310 "pddl+.yacc"
                                                                 {(yyval.t_goal)=(yyvsp[-1].t_goal);delete (yyvsp[-2].vtab);}
#line 4015 "pddl+.cpp"
    break;

  case 292: /* c_metric_spec: OPEN_BRAC METRIC c_optimization c_ground_f_exp CLOSE_BRAC  */
#line 1315 "pddl+.yacc"
       { (yyval.t_metric)= new metric_spec((yyvsp[-2].t_optimization),(yyvsp[-1].t_expression)); }
#line 4021 "pddl+.cpp"
    break;

  case 293: /* c_metric_spec: OPEN_BRAC METRIC error CLOSE_BRAC  */
#line 1317 "pddl+.yacc"
       {yyerrok; 
        log_error(E_FATAL,"Syntax error in metric declaration.");
        (yyval.t_metric)= NULL; }
#line 4029 "pddl+.cpp"
    break;

  case 294: /* c_length_spec: OPEN_BRAC LENGTH SERIAL INTVAL PARALLEL INTVAL CLOSE_BRAC  */
#line 1324 "pddl+.yacc"
       {(yyval.t_length_spec)= new length_spec(E_BOTH,(yyvsp[-3].ival),(yyvsp[-1].ival));}
#line 4035 "pddl+.cpp"
    break;

  case 295: /* c_length_spec: OPEN_BRAC LENGTH SERIAL INTVAL CLOSE_BRAC  */
#line 1327 "pddl+.yacc"
                {(yyval.t_length_spec) = new length_spec(E_SERIAL,(yyvsp[-1].ival));}
#line 4041 "pddl+.cpp"
    break;

  case 296: /* c_length_spec: OPEN_BRAC LENGTH PARALLEL INTVAL CLOSE_BRAC  */
#line 1331 "pddl+.yacc"
                {(yyval.t_length_spec) = new length_spec(E_PARALLEL,(yyvsp[-1].ival));}
#line 4047 "pddl+.cpp"
    break;

  case 297: /* c_optimization: MINIMIZE  */
#line 1337 "pddl+.yacc"
            {(yyval.t_optimization)= E_MINIMIZE;}
#line 4053 "pddl+.cpp"
    break;

  case 298: /* c_optimization: MAXIMIZE  */
#line 1338 "pddl+.yacc"
            {(yyval.t_optimization)= E_MAXIMIZE;}
#line 4059 "pddl+.cpp"
    break;

  case 299: /* c_ground_f_exp: OPEN_BRAC c_binary_ground_f_exp CLOSE_BRAC  */
#line 1343 "pddl+.yacc"
                                               {(yyval.t_expression)= (yyvsp[-1].t_expression);}
#line 4065 "pddl+.cpp"
    break;

  case 300: /* c_ground_f_exp: c_ground_f_head  */
#line 1344 "pddl+.yacc"
                    {(yyval.t_expression)= (yyvsp[0].t_func_term);}
#line 4071 "pddl+.cpp"
    break;

  case 301: /* c_ground_f_exp: c_number  */
#line 1345 "pddl+.yacc"
             {(yyval.t_expression)= (yyvsp[0].t_num_expression);}
#line 4077 "pddl+.cpp"
    break;

  case 302: /* c_ground_f_exp: TOTAL_TIME  */
#line 1346 "pddl+.yacc"
               { (yyval.t_expression)= new special_val_expr(E_TOTAL_TIME); }
#line 4083 "pddl+.cpp"
    break;

  case 303: /* c_ground_f_exp: OPEN_BRAC ISVIOLATED NAME CLOSE_BRAC  */
#line 1348 "pddl+.yacc"
                {(yyval.t_expression) = new violation_term((yyvsp[-1].cp));}
#line 4089 "pddl+.cpp"
    break;

  case 304: /* c_ground_f_exp: OPEN_BRAC TOTAL_TIME CLOSE_BRAC  */
#line 1349 "pddl+.yacc"
                                   { (yyval.t_expression)= new special_val_expr(E_TOTAL_TIME); }
#line 4095 "pddl+.cpp"
    break;

  case 305: /* c_binary_ground_f_exp: PLUS c_ground_f_exp c_binary_ground_f_pexps  */
#line 1353 "pddl+.yacc"
                                                  { (yyval.t_expression)= new plus_expression((yyvsp[-1].t_expression),(yyvsp[0].t_expression)); }
#line 4101 "pddl+.cpp"
    break;

  case 306: /* c_binary_ground_f_exp: HYPHEN c_ground_f_exp c_ground_f_exp  */
#line 1354 "pddl+.yacc"
                                         { (yyval.t_expression)= new minus_expression((yyvsp[-1].t_expression),(yyvsp[0].t_expression)); }
#line 4107 "pddl+.cpp"
    break;

  case 307: /* c_binary_ground_f_exp: MUL c_ground_f_exp c_binary_ground_f_mexps  */
#line 1355 "pddl+.yacc"
                                                  { (yyval.t_expression)= new mul_expression((yyvsp[-1].t_expression),(yyvsp[0].t_expression)); }
#line 4113 "pddl+.cpp"
    break;

  case 308: /* c_binary_ground_f_exp: DIV c_ground_f_exp c_ground_f_exp  */
#line 1356 "pddl+.yacc"
                                         { (yyval.t_expression)= new div_expression((yyvsp[-1].t_expression),(yyvsp[0].t_expression)); }
#line 4119 "pddl+.cpp"
    break;

  case 309: /* c_binary_ground_f_pexps: c_ground_f_exp  */
#line 1360 "pddl+.yacc"
                       {(yyval.t_expression) = (yyvsp[0].t_expression);}
#line 4125 "pddl+.cpp"
    break;

  case 310: /* c_binary_ground_f_pexps: c_ground_f_exp c_binary_ground_f_pexps  */
#line 1362 "pddl+.yacc"
        {(yyval.t_expression) = new plus_expression((yyvsp[-1].t_expression),(yyvsp[0].t_expression));}
#line 4131 "pddl+.cpp"
    break;

  case 311: /* c_binary_ground_f_mexps: c_ground_f_exp  */
#line 1366 "pddl+.yacc"
                       {(yyval.t_expression) = (yyvsp[0].t_expression);}
#line 4137 "pddl+.cpp"
    break;

  case 312: /* c_binary_ground_f_mexps: c_ground_f_exp c_binary_ground_f_mexps  */
#line 1368 "pddl+.yacc"
        {(yyval.t_expression) = new mul_expression((yyvsp[-1].t_expression),(yyvsp[0].t_expression));}
#line 4143 "pddl+.cpp"
    break;

  case 313: /* c_plan: c_step_t_d c_plan  */
#line 1374 "pddl+.yacc"
        {(yyval.t_plan)= (yyvsp[0].t_plan); 
         (yyval.t_plan)->push_front((yyvsp[-1].t_step)); }
#line 4150 "pddl+.cpp"
    break;

  case 314: /* c_plan: TIME FLOATVAL c_plan  */
#line 1377 "pddl+.yacc"
                {(yyval.t_plan) = (yyvsp[0].t_plan);(yyval.t_plan)->insertTime((yyvsp[-1].fval));}
#line 4156 "pddl+.cpp"
    break;

  case 315: /* c_plan: TIME INTVAL c_plan  */
#line 1379 "pddl+.yacc"
                {(yyval.t_plan) = (yyvsp[0].t_plan);(yyval.t_plan)->insertTime((yyvsp[-1].ival));}
#line 4162 "pddl+.cpp"
    break;

  case 316: /* c_plan: %empty  */
#line 1381 "pddl+.yacc"
        {(yyval.t_plan)= new plan;}
#line 4168 "pddl+.cpp"
    break;

  case 317: /* c_step_t_d: c_float COLON c_step_d  */
#line 1386 "pddl+.yacc"
        {(yyval.t_step)=(yyvsp[0].t_step); 
         (yyval.t_step)->start_time_given=1; 
         (yyval.t_step)->start_time=(yyvsp[-2].fval);}
#line 4176 "pddl+.cpp"
    break;

  case 318: /* c_step_t_d: c_step_d  */
#line 1390 "pddl+.yacc"
        {(yyval.t_step)=(yyvsp[0].t_step);
	 (yyval.t_step)->start_time_given=0;}
#line 4183 "pddl+.cpp"
    break;

  case 319: /* c_step_d: c_step OPEN_SQ c_float CLOSE_SQ  */
#line 1396 "pddl+.yacc"
        {(yyval.t_step)= (yyvsp[-3].t_step); 
	 (yyval.t_step)->duration_given=1;
         (yyval.t_step)->duration= (yyvsp[-1].fval);}
#line 4191 "pddl+.cpp"
    break;

  case 320: /* c_step_d: c_step  */
#line 1400 "pddl+.yacc"
        {(yyval.t_step)= (yyvsp[0].t_step);
         (yyval.t_step)->duration_given=0;}
#line 4198 "pddl+.cpp"
    break;

  case 321: /* c_step: OPEN_BRAC NAME c_const_symbols CLOSE_BRAC  */
#line 1406 "pddl+.yacc"
      {(yyval.t_step)= new plan_step( 
              current_analysis->op_tab.symbol_get((yyvsp[-2].cp)), 
	      (yyvsp[-1].t_const_symbol_list)); delete [] (yyvsp[-2].cp);
      }
#line 4207 "pddl+.cpp"
    break;

  case 322: /* c_float: FLOATVAL  */
#line 1413 "pddl+.yacc"
             {(yyval.fval)= (yyvsp[0].fval);}
#line 4213 "pddl+.cpp"
    break;

  case 323: /* c_float: INTVAL  */
#line 1414 "pddl+.yacc"
             {(yyval.fval)= (float) (yyvsp[0].ival);}
#line 4219 "pddl+.cpp"
    break;


#line 4223 "pddl+.cpp"

      default: break;
    }
  /* User semantic actions sometimes alter yychar, and that requires
     that yytoken be updated with the new translation.  We take the
     approach of translating immediately before every use of yytoken.
     One alternative is translating here after every semantic action,
     but that translation would be missed if the semantic action invokes
     YYABORT, YYACCEPT, or YYERROR immediately after altering yychar or
     if it invokes YYBACKUP.  In the case of YYABORT or YYACCEPT, an
     incorrect destructor might then be invoked immediately.  In the
     case of YYERROR or YYBACKUP, subsequent parser actions might lead
     to an incorrect destructor call or verbose syntax error message
     before the lookahead is translated.  */
  YY_SYMBOL_PRINT ("-> $$ =", YY_CAST (yysymbol_kind_t, yyr1[yyn]), &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;

  *++yyvsp = yyval;

  /* Now 'shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */
  {
    const int yylhs = yyr1[yyn] - YYNTOKENS;
    const int yyi = yypgoto[yylhs] + *yyssp;
    yystate = (0 <= yyi && yyi <= YYLAST && yycheck[yyi] == *yyssp
               ? yytable[yyi]
               : yydefgoto[yylhs]);
  }

  goto yynewstate;


/*--------------------------------------.
| yyerrlab -- here on detecting error.  |
`--------------------------------------*/
yyerrlab:
  /* Make sure we have latest lookahead translation.  See comments at
     user semantic actions for why this is necessary.  */
  yytoken = yychar == YYEMPTY ? YYSYMBOL_YYEMPTY : YYTRANSLATE (yychar);
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
      yyerror (YY_("syntax error"));
    }

  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
         error, discard it.  */

      if (yychar <= YYEOF)
        {
          /* Return failure if at end of input.  */
          if (yychar == YYEOF)
            YYABORT;
        }
      else
        {
          yydestruct ("Error: discarding",
                      yytoken, &yylval);
          yychar = YYEMPTY;
        }
    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:
  /* Pacify compilers when the user code never invokes YYERROR and the
     label yyerrorlab therefore never appears in user code.  */
  if (0)
    YYERROR;
  ++yynerrs;

  /* Do not reclaim the symbols of the rule whose action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;      /* Each real token shifted decrements this.  */

  /* Pop stack until we find a state that shifts the error token.  */
  for (;;)
    {
      yyn = yypact[yystate];
      if (!yypact_value_is_default (yyn))
        {
          yyn += YYSYMBOL_YYerror;
          if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYSYMBOL_YYerror)
            {
              yyn = yytable[yyn];
              if (0 < yyn)
                break;
            }
        }

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
        YYABORT;


      yydestruct ("Error: popping",
                  YY_ACCESSING_SYMBOL (yystate), yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", YY_ACCESSING_SYMBOL (yyn), yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturnlab;


/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturnlab;


/*-----------------------------------------------------------.
| yyexhaustedlab -- YYNOMEM (memory exhaustion) comes here.  |
`-----------------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  goto yyreturnlab;


/*----------------------------------------------------------.
| yyreturnlab -- parsing is finished, clean up and return.  |
`----------------------------------------------------------*/
yyreturnlab:
  if (yychar != YYEMPTY)
    {
      /* Make sure we have latest lookahead translation.  See comments at
         user semantic actions for why this is necessary.  */
      yytoken = YYTRANSLATE (yychar);
      yydestruct ("Cleanup: discarding lookahead",
                  yytoken, &yylval);
    }
  /* Do not reclaim the symbols of the rule whose action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
                  YY_ACCESSING_SYMBOL (+*yyssp), yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif

  return yyresult;
}

#line 1417 "pddl+.yacc"


#include <cstdio>
#include <iostream>
int line_no= 1;
using std::istream;
#include "lex.yy.cc"

namespace PDDL2UPMurphi_parser {
extern yyFlexLexer* yfl;
};


int yyerror(char * s)
{
    return 0;
}

int yylex()
{
    return yfl->yylex();
}
