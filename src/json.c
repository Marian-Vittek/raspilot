//////////////////////////////////////////////////////////////////////////

#include "common.h"

#if JSON_OBJECT_TREE
#include "sglib.h"
#define OBJCMPARATOR(f1,f2) strcmp(f1->u.name, f2->u.name)
typedef struct jsonFieldList orbtree;
SGLIB_DEFINE_RBTREE_PROTOTYPES(orbtree, left, right, nodeColor, OBJCMPARATOR);
SGLIB_DEFINE_RBTREE_FUNCTIONS(orbtree, left, right, nodeColor, OBJCMPARATOR);
#endif


char *jsonNodeTypeEnumNames[] = JSON_NODE_TYPE_ENUM_NAMES;
static char *jsonLiterals[] = {"false", "true", "null", NULL};

// forward decl
char *jsonConstantExpressionParse(char *s, struct jsonPosition *pp, double *res);


////////////////////////////////////////////////////////////////////////////

static char *jsonMaybeParseCppLineDirective(char *s, struct jsonPosition *pp) {
    char *d;

    while (*s == '#' && s[1] == ' ') {
	s += 2;
	pp->line = strtod(s, &s);
	while (*s != '\n' && *s != '"' && *s != 0) s ++;
	if (*s == '"') {
	    s ++;
	    d = s;
	    while (*s !='"' && *s != '\n' && *s != 0) s++;
	    if (*s == '"') {
		FREE(pp->file);
		pp->file = strnDuplicate(d, s-d);
		s++;
	    }
	}
	while (*s != '\n' && *s != 0) s++;
	if (*s == '\n') s++;
    }
    return(s);
}

static char *jsonPassNewline(char *s, struct jsonPosition *pp) {
    char *d;
    assert(*s == '\n');
    s ++;
    pp->line ++;
    if (JSON_ACCEPT_CPP_LINE_DIRECTIVES) {
	s = jsonMaybeParseCppLineDirective(s, pp);
    }
    return(s);
}

static char *jsonSkipBlankAndComments(char *s, struct jsonPosition *pp) {
    int 	cloopflag;

    if (s == NULL) return(NULL);
    
    cloopflag = 1;
    while (cloopflag == 1) {
	// skip blank
	while (isblank(*s) || *s == '\n' || *s == '\r' || *s == '\t') {
	    if (*s == '\n') s = jsonPassNewline(s, pp);
	    else s ++;
	}
	// skip comment
	if (s[0] == '/' && s[1] == '/') {
	    s += 2;
	    while (*s != '\n' && *s != 0) s++;
	    if (*s == 0) return(NULL);
	    s = jsonPassNewline(s, pp);
	} else if (s[0] == '/' && s[1] == '*') {
	    s += 2;
	    while (s[0] != '*' || s[1] != '/') {
		if (*s == 0) return(NULL);
		if (*s == '\n') s = jsonPassNewline(s, pp);
		else s ++;
	    }
	    s += 2;
	} else {
	    cloopflag = 0;
	}
    }
    return(s);
}


///////////////////////////////

static void jsonNotePosition(struct jsonnode *nn, struct jsonPosition *pp) {
#if JSON_SOURCE_POSITIONS
    nn->pos.file = strDuplicate(pp->file);
    nn->pos.line = pp->line;
#endif
}


#if 1
#include <math.h>
char *jsonCostantExpressionParseFunctonInvocation(char *s, struct jsonPosition *pp, double *res) {
    char 		*ss, *ff;
    double		arg;
    int			arity;
    
    ff = ss = s;
    while ((*ss >= 'a' && *ss <= 'z') || (*ss >= 'A' && *ss <= 'Z') || (*ss == '_')) ss ++;
    if (s == ss) return(NULL);
    if (*ss != '(') return(NULL);
    arity = 1;
    // only unary function for the moment
    s = jsonConstantExpressionParse(ss+1, pp, &arg);
    s = jsonSkipBlankAndComments(s, pp);
    if (*s != ')') return(NULL);
    s ++;
    if (strncmp(ff, "sin(", 4) == 0) {
	*res = sin(arg);
    } else if (strncmp(ff, "cos(", 4) == 0) {
	*res = cos(arg);
    } else if (strncmp(ff, "tan(", 4) == 0) {
	*res = tan(arg);
    } else if (strncmp(ff, "asin(", 4) == 0) {
	*res = asin(arg);
    } else if (strncmp(ff, "acos(", 4) == 0) {
	*res = acos(arg);
    } else if (strncmp(ff, "atan(", 4) == 0) {
	*res = atan(arg);
    } else if (strncmp(ff, "exp(", 4) == 0) {
	*res = exp(arg);
    } else if (strncmp(ff, "exp2(", 4) == 0) {
	*res = exp2(arg);
    } else if (strncmp(ff, "log(", 4) == 0) {
	*res = log(arg);
    } else if (strncmp(ff, "log2(", 4) == 0) {
	*res = log2(arg);
    }
    return(s);
}
#endif

char *jsonConstantExpressionParseTerm(char *s, struct jsonPosition *pp, double *res) {
    s = jsonSkipBlankAndComments(s, pp);
    if (s == NULL) return(NULL);
    if (isdigit(*s)) {
	*res = strtod(s, &s);
    } else if (*s == '-') {
        s = jsonConstantExpressionParseTerm(s+1, pp, res);
	*res = - *res;
    } else if (*s == '(') {
        s = jsonConstantExpressionParse(s+1, pp, res);
	s = jsonSkipBlankAndComments(s, pp);
        if (s == NULL || *s != ')') return(NULL);
        s++;
    } else {
	s = jsonCostantExpressionParseFunctonInvocation(s, pp, res);
    }
    return(s);
}

char *jsonConstantExpressionParseMulDiv(char *s, struct jsonPosition *pp, double *res) {
    double r;
    s = jsonConstantExpressionParseTerm(s, pp, res);
    for (;;) {
	s = jsonSkipBlankAndComments(s, pp);
	if (s == NULL) return(NULL);
        if (*s == '*') {
            s = jsonConstantExpressionParseTerm(s+1, pp, &r);
	    *res *= r;
        } else if (*s == '/') {
            s = jsonConstantExpressionParseTerm(s+1, pp, &r);
	    *res /= r;
        } else {
	    break;
	}
    }
    return(s);
}

char *jsonConstantExpressionParseAddSub(char *s, struct jsonPosition *pp, double *res) {
    double r;
    s = jsonConstantExpressionParseMulDiv(s, pp, res);
    for (;;) {
	s = jsonSkipBlankAndComments(s, pp);
	if (s == NULL) return(NULL);
        if (*s == '+') {
            s = jsonConstantExpressionParseMulDiv(s+1, pp, &r);
	    *res += r;
        } else if (*s == '-') {
            s = jsonConstantExpressionParseMulDiv(s+1, pp, &r);
	    *res -= r;
        } else {
	    break;
	}
    }
    return(s);
}

char *jsonConstantExpressionParse(char *s, struct jsonPosition *pp, double *res) {
    return(jsonConstantExpressionParseAddSub(s, pp, res));
}

///////////////////////////

char *jsonParseStringLiteral(char *ss, char **res) {
    char			*d;
    char			*s;
    int				i, dsize;
    unsigned long	cc;

    dsize = 32;
    ALLOCC(d, dsize, char);
    for(s=ss,i=0; *s != 0 && *s != '"'; s++,i++) {
	if (i >= dsize) {
	    dsize *= 2;
	    REALLOCC(d, dsize, char);
	}
	if (*s == '\\') {
	    s++;
	    if (*s == 'b') {
		d[i] = '\b';
	    } else if (*s == 'f') {
		d[i] = '\f';
	    } else if (*s == 'n') {
		d[i] = '\n';
	    } else if (*s == 'r') {
		d[i] = '\r';
	    } else if (*s == 't') {
		d[i] = '\t';
	    } else if (*s == 'u') {
		cc = 0;
		s ++; if (isxdigit(*s)) cc = cc * 16 + hexDigitCharToInt(*s); if (*s == 0 || *s == '"') goto jsonBreak;
		s ++; if (isxdigit(*s)) cc = cc * 16 + hexDigitCharToInt(*s); if (*s == 0 || *s == '"') goto jsonBreak;
		s ++; if (isxdigit(*s)) cc = cc * 16 + hexDigitCharToInt(*s); if (*s == 0 || *s == '"') goto jsonBreak;
		s ++; if (isxdigit(*s)) cc = cc * 16 + hexDigitCharToInt(*s); if (*s == 0 || *s == '"') goto jsonBreak;
		d[i] = cc;
	    } else {
		d[i] = *s;
	    }
	} else {
	    d[i] = *s;
	}
    }

jsonBreak:
    if (*s == 0) {
	FREE(d);
	*res = NULL;
	return(NULL);
    }
    REALLOCC(d, i+1, char);
    d[i] = 0;
    *res = d;
    return(s);
}

char *jsonParseStringRec(char *b, struct jsonPosition *pp, struct jsonnode **res) {
    char    			*s, *ss, *send, *key, *kkey, *field, *ll, *slit, *dlit;
    int     			i, index, ec, fieldIdLen, fieldLen, llen;
    int     			fieldId, jclen, keylen;
    struct jsonnode 		*rrr, *eee, **table;
    struct jsonFieldList	*ff, *ffnext, **fff;
    struct jsonPosition		spp;
    double			dval;
    
    s = b;

    // skip comments if any
    s = jsonSkipBlankAndComments(s, pp);
    spp = *pp;
	
    // printf("\nParsing '%s'\n", b);
    if (*s == '{') {
        // JSON Object
	CALLOC(*res, struct jsonnode);
	(*res)->type = JSON_NODE_TYPE_OBJECT;
	jsonNotePosition(*res, &spp);
	(*res)->u.fields = NULL;
#if JSON_OBJECT_TREE
	(*res)->u.t.fields = NULL;
	(*res)->u.t.tree = NULL;
#endif
	fff = &(*res)->u.fields;
        do {
            s++;
            s = jsonSkipBlankAndComments(s, pp);
	    // following line allows comma after the last record
            if (*s == '}') break;
	    if (*s == '"') {
		s = jsonParseStringLiteral(s+1, &kkey);
		if (s == NULL) goto jsonError;
		s++;
	    } else {
		// This allows extension where object keys are not inside quotes
		key = s;
		while (*s != ':' && ! isblank(*s) && *s!=0) s++;
		if (*s == 0) goto jsonError;
		keylen = s-key;
		ALLOCC(kkey, keylen+1, char);
		strncpy(kkey, key, keylen);
		kkey[keylen] = 0;		
	    }
	    s = jsonSkipBlankAndComments(s, pp);
            if (*s != ':') {
		FREE(kkey);
		goto jsonError;
	    }
            s++;
            s = jsonSkipBlankAndComments(s, pp);
            s = jsonParseStringRec(s, pp, &rrr);
	    if (s == NULL) {
		FREE(kkey);
		return(NULL);
	    }
            s = jsonSkipBlankAndComments(s, pp);
	    CALLOC(*fff, struct jsonFieldList);
	    (*fff)->u.name = kkey;
	    (*fff)->val = rrr;
	    (*fff)->next = NULL;
#if JSON_OBJECT_TREE
	    sglib_orbtree_add(&(*res)->u.t.tree, *fff);
#endif
	    fff = &(*fff)->next;
        } while (*s == ',');
        if (*s != '}') goto jsonError;
        s ++;
    } else if (*s == '[') {
        // JSON array
	CALLOC(*res, struct jsonnode);
	(*res)->type = JSON_NODE_TYPE_ARRAY;
	jsonNotePosition(*res, &spp);
	(*res)->u.fields = NULL;
	fff = &(*res)->u.fields;
	index = 0;
        do {
            s++;
            s = jsonSkipBlankAndComments(s, pp);
	    if (*s == ']') {
		// This allows comma after last element
		eee = NULL;
	    } else {
		s = jsonParseStringRec(s, pp, &eee);
		if (s == NULL) return(NULL);
		s = jsonSkipBlankAndComments(s, pp);
	    }
	    if (eee != NULL) {
		CALLOC(*fff, struct jsonFieldList);
		(*fff)->u.index = index;
		(*fff)->val = eee;
		(*fff)->next = NULL;
		fff = &(*fff)->next;
		index ++;
	    }
        } while (*s == ',');
        if (*s != ']') goto jsonError;
#if JSON_ARRAY_TABLE || JSON_ARRAY_TABLE_ONLY
	ALLOCC(table, index, struct jsonnode *);
	i=0;
	ff=(*res)->u.fields;
	while (i<index && ff!=NULL) {
	    assert(ff->u.index == i);
	    table[i] = ff->val;
	    ffnext = ff->next;
#if JSON_ARRAY_TABLE_ONLY
	    FREE(ff);
#endif	    
	    i++;
	    ff=ffnext;
	}
	assert(i==index && ff == NULL);
#if ! JSON_ARRAY_TABLE_ONLY
	(*res)->u.a.fields = (*res)->u.fields;
#endif
	(*res)->u.a.tableSize = index;
	(*res)->u.a.table = table;
#endif
        s++;
    } else if (*s == '"') {
        // JSON string
        s ++;
	ss = jsonParseStringLiteral(s, &slit);
        if (ss == NULL) goto jsonError;
        s = ss+1;
	CALLOC(*res, struct jsonnode);
	(*res)->type = JSON_NODE_TYPE_STRING;
	jsonNotePosition(*res, &spp);
	(*res)->u.s = slit;
    } else if (isdigit(*s) || *s == '.' || *s == '+' || *s == '-' || *s == '(') {
	s = jsonConstantExpressionParse(s, pp, &dval);
	if (s == NULL) goto jsonError;
	CALLOC(*res, struct jsonnode);
	(*res)->type = JSON_NODE_TYPE_NUMBER;
	jsonNotePosition(*res, &spp);
	(*res)->u.n = dval;
    } else {
        // Nothing of above, it shall be a literal
        for(i=0; jsonLiterals[i]!=NULL; i++) {
            ll = jsonLiterals[i];
            llen = strlen(ll);		// can be precomputed
            if (strncmp(s, ll, llen) == 0 && ! isalpha(s[llen+1])) {
                s += llen;
                break;
            }
        }
        // If it is even not a literal, make error
        if (jsonLiterals[i]==NULL) goto jsonError;
	if (strcmp(jsonLiterals[i], "null") == 0) {
	    *res = NULL;
	} else {
	    CALLOC(*res, struct jsonnode);
	    (*res)->type = JSON_NODE_TYPE_BOOL;
	    jsonNotePosition(*res, &spp);
	    (*res)->u.b = i;
	}
    }

    return(s);

jsonError:
    printf("%s: Error: Json syntax error at %s:%d.\n", PPREFIX(), pp->file, pp->line);
    return(NULL);
}

char *jsonParseString(char *jsonString, char *inputName, struct jsonnode **res) {
    struct jsonPosition position;
    char		*s;
    
    position.line = 1;
    if (inputName == NULL) inputName = "";
    position.file = strDuplicate(inputName);
    jsonString = jsonMaybeParseCppLineDirective(jsonString, &position);
    s = jsonParseStringRec(jsonString, &position, res);
    if (s == NULL) {
	jsonFree(*res);
	*res = NULL;
    }
    FREE(position.file);
    return(s);
}

void jsonFree(struct jsonnode *nn) {
    int i;
    struct jsonFieldList    *ll, *llnext;

    if (nn == NULL) return;

    switch (nn->type) {
    case JSON_NODE_TYPE_STRING:
	FREE(nn->u.s);
	break;
    case JSON_NODE_TYPE_ARRAY:
#if JSON_ARRAY_TABLE || JSON_ARRAY_TABLE_ONLY
	for(i=0; i<nn->u.a.tableSize; i++) FREE(nn->u.a.table[i]);
	FREE(nn->u.a.table);
	break;
#endif
	// if not table, passthrough to freeing list of elements!
    case JSON_NODE_TYPE_OBJECT:
	ll = nn->u.fields; 
	while (ll != NULL) {
	    if (nn->type == JSON_NODE_TYPE_OBJECT) FREE(ll->u.name);
	    jsonFree(ll->val);
	    llnext = ll->next;
	    FREE(ll);
	    ll = llnext;
	}
	break;
    }

#if JSON_SOURCE_POSITIONS
    FREE(nn->pos.file);
#endif
    FREE(nn);
}

void jsonPrintString(char *ss, FILE *ff) {
    int		i;
    char	*s;

    putc('"', ff);
    for(s=ss; *s; s++) {
	switch (*s) {
	case '"':
	    putc('\\', ff);	putc('"', ff);
	    break;
	case '\\':
	    putc('\\', ff);	putc('\\', ff);
	    break;
	case '\b':
	    putc('\\', ff);	putc('b', ff);
	    break;
	case '\f':
	    putc('\\', ff);	putc('f', ff);
	    break;
	case '\n':
	    putc('\\', ff);	putc('n', ff);
	    break;
	case '\r':
	    putc('\\', ff);	putc('r', ff);
	    break;
	case '\t':
	    putc('\\', ff);	putc('t', ff);
	    break;
	default:
	    if (isprint(*s)) {
		putc(*s, ff);
	    } else {
		fprintf(ff, "u%04x", *s);
	    }
	}
    }
    putc('"', ff);
}

void jsonPrint(struct jsonnode *nn, FILE *ff) {
    int						i;
    char					*ss;
    struct jsonFieldList	*ll;

    if (nn == NULL) {
	fprintf(ff, "null");
	return;
    }
	
    switch (nn->type) {
    case JSON_NODE_TYPE_BOOL:
	if (nn->u.b) fprintf(ff, "true");
	else fprintf(ff, "false");
	break;
    case JSON_NODE_TYPE_NUMBER:
	fprintf(ff, "%f", nn->u.n);
	break;
    case JSON_NODE_TYPE_STRING:
	jsonPrintString(nn->u.s, ff);
	break;
    case JSON_NODE_TYPE_ARRAY:
	fprintf(ff, "[");
	for(ll=nn->u.fields; ll!=NULL; ll=ll->next) {
	    jsonPrint(ll->val, ff);
	    fprintf(ff, ",\n");
	}
	fprintf(ff, "]");
	break;
    case JSON_NODE_TYPE_OBJECT:
	fprintf(ff, "{");
	for(ll=nn->u.fields; ll!=NULL; ll=ll->next) {
	    fprintf(ff, "\"%s\": ", ll->u.name);
	    jsonPrint(ll->val, ff);
	    fprintf(ff, ",\n");
	}
	fprintf(ff, "}");
	break;
    }
}

struct jsonnode *jsonFindArrayIndex(struct jsonnode *nn, int index) {
    struct jsonFieldList	*ll, memb;
    int				i;
    
    if (nn == NULL || nn->type != JSON_NODE_TYPE_ARRAY || index < 0) return(NULL);
#if JSON_ARRAY_TABLE || JSON_ARRAY_TABLE_ONLY
    if (index >= nn->u.a.tableSize) return(NULL);
    return(nn->u.a.table[index]);
#else
    for(i=0,ll=nn->u.fields; i<index && ll!=NULL; i++,ll=ll->next) ;
#endif
    if (ll == NULL) return(NULL);
    return(ll->val);
}

struct jsonnode *jsonFindObjectField(struct jsonnode *nn, char *name) {
    struct jsonFieldList	*ll, memb;

    if (nn == NULL || nn->type != JSON_NODE_TYPE_OBJECT) return(NULL);
#if JSON_OBJECT_TREE
    memb.u.name = name;
    ll = sglib_orbtree_find_member(nn->u.t.tree, &memb);
#else
    for(ll=nn->u.fields; ll!=NULL; ll=ll->next) {
	if (strcmp(ll->u.name, name) == 0) break;
    }
#endif
    if (ll == NULL) return(NULL);
    return(ll->val);
}

// go through parsed json and find composed name like "field1.field2[42].field3"
struct jsonnode *jsonFind(struct jsonnode *nn, char *composedField) {
    int 	i, flen;
    char	*cc, *ee;
    static char	*field = NULL;
    
    if (nn == NULL || composedField == NULL) return(nn);
    
    cc = composedField;
    while (isspace(*cc)) cc ++;
    if (*cc == 0) return(nn);
    
    if (*cc == '[') {
	// TODO: Test this, it has not been used yet
	cc ++;
	if (nn->type != JSON_NODE_TYPE_ARRAY) return(NULL);
	i = strtol(cc, &ee, 10);
	if (ee == cc) return(NULL);
	cc = ee;
	while (isspace(*cc)) cc ++;
	if (*cc != ']') return(NULL);
	cc ++;
	// tail recursion
	return(jsonFind(jsonFindArrayIndex(nn, i), cc));
    } else {
	if (nn->type != JSON_NODE_TYPE_OBJECT) return(NULL);
	for(ee = cc; *ee != 0; ee++) {
	    if (*ee == '.' || *ee == '[' || isspace(*ee)) break;
	}
	flen = ee-cc;
	REALLOCC(field, flen+1, char);
	strncpy(field, cc, flen);
	field[flen] = 0;
	cc = ee;
	while (isspace(*cc)) cc ++;
	if (*cc == '.') cc ++;
	// tail recursion
	return(jsonFind(jsonFindObjectField(nn, field), cc));
    }
}

double jsonFindDouble(struct jsonnode *nn, char *name, double defaultValue) {
    struct jsonnode 	*tt;

    tt = jsonFind(nn, name);
    if (tt == NULL || tt->type != JSON_NODE_TYPE_NUMBER) return(defaultValue);
    return(tt->u.n);
}

char *jsonFindString(struct jsonnode *nn, char *name, char *defaultValue) {
    struct jsonnode 	*tt;

    tt = jsonFind(nn, name);
    if (tt == NULL || tt->type != JSON_NODE_TYPE_STRING) return(defaultValue);
    return(tt->u.s);
}

struct jsonFieldList *jsonFindFieldList(struct jsonnode *nn, char *name, int jsonType) {
    struct jsonnode 	*tt;

    tt = jsonFind(nn, name);
    if (tt == NULL || tt->type != jsonType) return(NULL);
    return(tt->u.fields);
}

