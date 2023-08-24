#defconfig
make ARCH=arm64 CROSS_COMPILE=/opt/toolchain/7.5.0/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- anlogic_dr1m90_openamp_example_defconfig

#dtbs
make ARCH=arm64 CROSS_COMPILE=/opt/toolchain/7.5.0/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- dtbs

#Image
make ARCH=arm64 CROSS_COMPILE=/opt/toolchain/7.5.0/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- -j8

#gzip
mkimage -A arm64 -O linux -T kernel -C gzip -a 0x00400000 -e 0x00400000 -n Linux -d arch/arm64/boot/Image.gz arch/arm64/boot/uImage.gz

#lz4
lz4 arch/arm64/boot/Image arch/arm64/boot/Image.lz4 -f -9
mkimage -A arm64 -O linux -T kernel -C lz4 -a 0x00400000 -e 0x00400000 -n Linux -d arch/arm64/boot/Image.lz4 arch/arm64/boot/uImage.lz4


