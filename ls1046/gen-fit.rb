#!/usr/bin/env ruby

$path = "output/build_arm64_standalone/images"
$output = "fit-ls1046.itb"
$pwd = %x(pwd).chomp()

def create_fit()
    its = '''
/dts-v1/;

/ {
	description = "FIT image file";
	#address-cells = <1>;

	images {
		kernel {
			description = "Kernel";
			data = /incbin/("PATH_KERNEL");
			type = "kernel";
			arch = "arm64";
			os = "linux";
			compression = "gzip";
			load = <0x80080000>;
			entry = <0x80080000>;
		};

		fdt{
			description = "Flattened Device Tree blob";
			data = /incbin/("PATH_FDT");
			type = "flat_dt";
			arch = "arm64";
			load = <0x90000000>;
			compression = "none";
		};

		fdt_sr{
			description = "Flattened Device Tree blob";
			data = /incbin/("PATH_FDT_SR");
			type = "flat_dt";
			arch = "arm64";
			load = <0x90000000>;
			compression = "none";
		};

		ramdisk {
			description = "ramdisk";
			data = /incbin/("PATH_RAMDISK");
			type = "ramdisk";
			arch = "arm64";
			os = "linux";
			load = <0x88080000>;
			compression = "none";
		};
	};

	configurations {
	        default = "6813_0@ls1046";

		6813_0@ls1046 {
			description = "ls1046 kernel";
			kernel = "kernel";
			fdt = "fdt";
			ramdisk = "ramdisk";
		};

		conf@ls1046_sr {
			description = "ls1046 sunrise kernel";
			kernel = "kernel";
			fdt = "fdt_sr";
			ramdisk = "ramdisk";
		};

	};
};
    '''
    its.sub! "PATH_KERNEL", "#{$pwd}/#{$path}/kernel.bin.gz"
    its.sub! "PATH_FDT", "#{$pwd}/#{$path}/mchp-ls1046a-lan966x_ad.dtb"
    its.sub! "PATH_FDT_SR", "#{$pwd}/#{$path}/mchp-ls1046a-lan966x_sr.dtb"
    its.sub! "PATH_RAMDISK", "#{$pwd}/#{$path}/rootfs.squashfs"

    File.open("#{$path}/vmlinux.its", 'w') { |file| file.write(its) }

    system("mkimage -f #{$pwd}/#{$path}/vmlinux.its #{$path}/#{$output}")
end

# make a working copy
system("cp #{$path}/mscc-linux-kernel.bin.xz #{$path}/kernel.bin.xz")

# extract
system("xz -d #{$path}/kernel.bin.xz")

# gzip kernel
system("gzip -f #{$path}/kernel.bin")

# create fit
create_fit()

# clean after ourselfs
system("rm #{$path}/kernel.bin.gz")

# copy the uboot file that contains also the spl and remove the one
# that is already there
system("rm #{$path}/u-boot.bin")
system("cp #{$path}/../build/uboot-custom/u-boot-with-spl-pbl.bin #{$path}/")
