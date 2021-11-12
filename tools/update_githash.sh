#!/bin/sh

GITHASH=$(git rev-parse --short HEAD | tr 'a-z' 'A-Z')

cat << EOF > modules/githash.h

#ifndef __GITHASH_H__
#define __GITHASH_H__

#define GITHASH "$GITHASH"

#endif
EOF
