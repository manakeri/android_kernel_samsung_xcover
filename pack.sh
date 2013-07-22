#!/bin/bash -x

tar -H ustar -cf zImage.tar zImage
md5sum -t zImage.tar >> zImage.tar
mv zImage.tar zImage.tar.md5
