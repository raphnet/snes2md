#!/bin/bash

PREFIX=snes2md

echo "Release script for $PREFIX"

printHelpAndExit()
{
	echo "Syntax: ./release.sh version releasedir"
	echo
	echo "ex: './release 1.0' will produce $PREFIX-1.0.tar.gz in releasedir out of git HEAD,"
	echo "untar it, build the firmware and create $PREFIX-1.0.hex in releasedir."
	exit
}

# arg1: Makefile to use
# arg2: Output hex file
# arg3: Output file name in parent directory
#
# Call from project directory.
buildHex()
{
	make -f $1 clean
	make -f $1
	cp $2 ../$3
}

if [ $# -ne 2 ]; then
	printHelpAndExit
fi


VERSION=$1
RELEASEDIR=$2
DIRNAME=$PREFIX-$VERSION
FILENAME=$PREFIX-$VERSION.tar.gz
TAG=v$VERSION

echo "Version: $VERSION"
echo "Filename: $FILENAME"
echo "Release directory: $RELEASEDIR"
echo "--------"
echo "Ready? Press ENTER to go ahead (or CTRL+C to cancel)"

read

if [ -f $RELEASEDIR/$FILENAME ]; then
	echo "Release file already exists!"
	exit 1
fi

git tag $TAG -f -a
git archive --format=tar --prefix=$DIRNAME/ HEAD | gzip > $RELEASEDIR/$FILENAME

cd $RELEASEDIR
tar zxf $FILENAME

# Enter project directory and build release
cd $DIRNAME
buildHex Makefile snes2md.hex $PREFIX.m8.$VERSION.hex
buildHex Makefile.m168 snes2md.hex $PREFIX.m168.$VERSION.hex

cd ..
echo
echo
echo
ls -l $PREFIX*$VERSION.*

echo "--------"
echo "Done."
