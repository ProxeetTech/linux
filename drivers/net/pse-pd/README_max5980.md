# max5980
Linux driver for MAX5980 IEEE 802.3at/af Quad Port Power-over-Ethernet PSE Controller.

Bindings example:

max5980: max5980@20 {
        compatible = "maxim,max5980";
        reg = <0x20>;
        irq-gpio = <84>;
        port0@0 {
                enable = <1>;
                mode = "auto";
                power = <1>;
        };
        port1@1 {
                enable = <1>;
                mode = "manual";
                power = <0>;
        };
        port2@2 {
                enable = <0>;
        };
        port3@3 {
                enable = <0>;
        };
};
