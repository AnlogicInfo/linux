#defconfig
#make ARCH=riscv CROSS_COMPILE=../toolchain/riscv/gcc/bin/riscv-nuclei-linux-gnu- al9000_rpu_defconfig
cp arch/riscv/configs/anlogic_al9000_rpu_defconfig .config
make ARCH=riscv CROSS_COMPILE=../toolchain/riscv/gcc/bin/riscv-nuclei-linux-gnu- olddefconfig

#make dtbs
make ARCH=riscv CROSS_COMPILE=../toolchain/riscv/gcc/bin/riscv-nuclei-linux-gnu- dtbs

#Image
make ARCH=riscv CROSS_COMPILE=../toolchain/riscv/gcc/bin/riscv-nuclei-linux-gnu- Image -j32

#gzip
mkimage -A riscv -O linux -T kernel -C gzip -a 0x00400000 -e 0x00400000 -n Linux -d arch/riscv/boot/Image.gz arch/riscv/boot/uImage.gz

#lz4
lz4 arch/riscv/boot/Image arch/riscv/boot/Image.lz4 -f -9
mkimage -A riscv -O linux -T kernel -C lz4 -a 0x00400000 -e 0x00400000 -n Linux -d arch/riscv/boot/Image.lz4 arch/riscv/boot/uImage.lz4

