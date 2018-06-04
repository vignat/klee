// Copied from uclibc

/*
 * Copyright (C) 2002     Manuel Novoa III
 * Copyright (C) 2000-2005 Erik Andersen <andersen@uclibc.org>
 *
 * Licensed under the LGPL v2.1, see the file COPYING.LIB in this tarball.
 */

#include <stdlib.h>

void *memchr(const void *s, int c, size_t n)
{
	register const unsigned char *r = (const unsigned char *) s;

	while (n) {
		if (*r == ((unsigned char)c)) {
			return (void *) r;	/* silence the warning */
		}
		++r;
		--n;
	}

	return NULL;
}
