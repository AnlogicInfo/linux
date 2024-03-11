#defconfig
make ARCH=riscv CROSS_COMPILE=/opt/toolchain/riscv-nuclei-linux-gnu-2020.08/bin/riscv-nuclei-linux-gnu- anlogic_dr1v90_defconfig

#make dtbs
make ARCH=riscv CROSS_COMPILE=/opt/toolchain/riscv-nuclei-linux-gnu-2020.08/bin/riscv-nuclei-linux-gnu- dtbs

#Image
make ARCH=riscv CROSS_COMPILE=/opt/toolchain/riscv-nuclei-linux-gnu-2020.08/bin/riscv-nuclei-linux-gnu- Image -j32

#gzip
#gzip -f -9 arch/riscv/boot/Image
#mkimage -A riscv -O linux -T kernel -C gzip -a 0x00400000 -e 0x00400000 -n Linux -d arch/riscv/boot/Image.gz arch/riscv/boot/uImage.gz

#lz4
lz4 arch/riscv/boot/Image arch/riscv/boot/Image.lz4 -f -9
mkimage -A riscv -O linux -T kernel -C lz4 -a 0x00400000 -e 0x00400000 -n Linux -d arch/riscv/boot/Image.lz4 arch/riscv/boot/uImage.lz4

