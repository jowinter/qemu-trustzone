#include "sysemu.h"

/* NOR flash devices */
typedef struct pflash_t pflash_t;

/* pflash_cfi01.c */
pflash_t *pflash_cfi01_register(target_phys_addr_t base, ram_addr_t off,
                                BlockDriverState *bs,
                                uint32_t sector_len, int nb_blocs, int width,
                                uint16_t id0, uint16_t id1,
                                uint16_t id2, uint16_t id3, int be);

/* pflash_cfi02.c */
pflash_t *pflash_cfi02_register(target_phys_addr_t base, ram_addr_t off,
                                BlockDriverState *bs, uint32_t sector_len,
                                int nb_blocs, int nb_mappings, int width,
                                uint16_t id0, uint16_t id1,
                                uint16_t id2, uint16_t id3,
                                uint16_t unlock_addr0, uint16_t unlock_addr1,
                                int be);

/* nand.c */
DeviceState *nand_init(int manf_id, int chip_id, BlockDriverState *bs);
void nand_setpins(DeviceState *dev,
                int cle, int ale, int ce, int wp, int gnd);
void nand_getpins(DeviceState *dev, int *rb);
void nand_setio(DeviceState *dev, uint32_t value);
uint32_t nand_getio(DeviceState *dev);
uint32_t nand_getbuswidth(DeviceState *dev);

#define NAND_MFR_TOSHIBA	0x98
#define NAND_MFR_SAMSUNG	0xec
#define NAND_MFR_FUJITSU	0x04
#define NAND_MFR_NATIONAL	0x8f
#define NAND_MFR_RENESAS	0x07
#define NAND_MFR_STMICRO	0x20
#define NAND_MFR_HYNIX		0xad
#define NAND_MFR_MICRON		0x2c

/* onenand.c */
DeviceState *onenand_create(uint16_t man_id, uint16_t dev_id, uint16_t ver_id,
                            int regshift, qemu_irq irq, BlockDriverState *bs);
void *onenand_raw_otp(DeviceState *onenand_device);

/* ecc.c */
typedef struct {
    uint8_t cp;		/* Column parity */
    uint16_t lp[2];	/* Line parity */
    uint16_t count;
} ECCState;

uint8_t ecc_digest(ECCState *s, uint8_t sample);
void ecc_reset(ECCState *s);
void ecc_put(QEMUFile *f, ECCState *s);
void ecc_get(QEMUFile *f, ECCState *s);
