#ifndef FW_HEADER_H
#define FW_HEADER_H

#include <stdbool.h>
#include <stdint.h>

enum
{
	FW_PRELDR = 0,
	FW_LDR,
	FW_APP,

	FW_COUNT
};
enum
{
	LOCK_NONE = 0,
	LOCK_BY_CRC,
	LOCK_BY_ADDR,
	LOCK_BY_SIZE_SMALL,
	LOCK_BY_ZERO_FIELDS_COUNT,
	LOCK_NO_PROD_FIELD,
	LOCK_NO_PROD_NAME_FIELD,
	LOCK_PROD_MISMATCH,
	LOCK_PROD_NAME_FAULT,
	LOCK_FLASH_DOUBLE_ERROR, // flash ECC double-error
} FW_HDR_LOCK_t;

///////////////////////// New ///////////////////////

typedef struct
{
	const char *const key;
	const char *const val;
} field_t;

typedef struct
{
	uint32_t magic_fw_code;
	uint32_t fw_size;
	uint32_t fw_crc;
	union
	{
#if __INTPTR_WIDTH__ == 32
		const field_t *const fields;
#endif
		uint32_t p32;
	};
} header_t;

const field_t fileds[] =
	{
		{"ver", "one"},
		{"c", "d"},
		{0}
	};

const header_t header __attribute__((section(".header"))) =
	{
		.magic_fw_code = 0xaabbccdd,
#if __INTPTR_WIDTH__ == 32
		.fields = fileds
#endif
};

///////////////////////// New ///////////////////////

typedef struct
{
	int locked;							// locked (::FW_HDR_LOCK_t)
	uintptr_t addr;						// pointer to fw
	int fields_count;					// count of fields
	const char *field_product_ptr;		// pointer to value of the "product" field
	int field_product_len;				// "product" field length
	const char *field_product_name_ptr; // pointer to value of the "product_name" field
	int field_product_name_len;			// "product_name" field length
	uint32_t ver_major;					// parsed major version
	uint32_t ver_minor;					// parsed minor version
	uint32_t ver_patch;					// parsed patch version
	uint32_t ver_compat;				// parsed compatibility version
} fw_info_t;

int fw_fields_get_count(uint32_t addr_fw_start, uint32_t region_size);
const char *fw_fields_find_by_key(uint32_t addr_fw_start, const char *key, uint32_t region_size);
const char *fw_fields_find_by_key_helper(fw_info_t *fw, const char *key);
bool fw_fields_find_by_iterator(uint32_t addr_fw_start, int iterator, const char **p_key, const char **p_value, uint32_t region_size);
bool fw_fields_find_by_iterator_helper(fw_info_t *fw, int iterator, const char **p_key, const char **p_value);

int str_len_safe(const char *s);
bool str_compare_equal_safe_two_arg(const char *s1, const char *s2);
bool str_compare_equal_safe_first_arg(const char *s1, const char *s2); // hack
bool flash_check_range(uint32_t addr_start, uint32_t size);

bool fw_header_check_region(fw_info_t *fw, uint32_t header_offset, uint32_t max_size);
void fw_header_check_all(void);

extern fw_info_t g_fw_info[FW_COUNT];

#endif // FW_HEADER_H