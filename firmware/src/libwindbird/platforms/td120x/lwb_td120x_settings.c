#include "../../libwindbird.h"

#ifdef LWB_PLATFORM_TD120X

#include <em_msc.h>

#include <string.h>

#include "../../core/lwb_settings.h"
#include "../../core/lwb_serial.h"

#if MODULE_REVISION != REVISION_TD1208
    #error "FLASH_START_ADDR must be updated for this platform"
#endif

#define FLASH_START_ADDR 0x0001FC00

#define FLASH_PAGE_SIZE 512

#define FLASH_MAX_WORDS (FLASH_PAGE_SIZE/sizeof(uint32_t))

static uint32_t buffer[FLASH_MAX_WORDS];

static bool isEditing = false;

// For simplicity, we use only a single page
// this allows us to store 512 / 4 = 128 values
// this should be sufficient

void BeginEdit() {
	memcpy(buffer, (uint32_t *)FLASH_START_ADDR, FLASH_PAGE_SIZE);
	isEditing = true;
}

bool LWB_SETTINGS_Set(uint8_t index, uint32_t val) {
	if (index < 0 || index >= FLASH_MAX_WORDS) {
		LWB_SERIAL_Printfln("FLASH: Word %d is out of range", index);
		return false;
	}
	if (!isEditing) BeginEdit();
	buffer[index] = val;
	return true;
}

bool LWB_SETTINGS_SetFloat(uint8_t index, float val) {
	return LWB_SETTINGS_Set(index, *(uint32_t *)&val);
}

bool LWB_SETTINGS_Clear() {
	MSC_Init();
	if (MSC_ErasePage((uint32_t *)FLASH_START_ADDR) != mscReturnOk) {
		LWB_SERIAL_Println("FLASH: Erase failed");
		MSC_Deinit();
		return false;
	}
	MSC_Deinit();
	isEditing = false;
	memset(buffer, 0xFFFFFFFF, FLASH_MAX_WORDS);
	return true;
}

bool LWB_SETTINGS_Save() {
	if (!isEditing) return true;

	MSC_Init();

	if (MSC_ErasePage((uint32_t *)FLASH_START_ADDR) != mscReturnOk) {
		LWB_SERIAL_Println("FLASH: Erase failed");
		MSC_Deinit();
		return false;
	}

	if (MSC_WriteWord((uint32_t *)FLASH_START_ADDR, buffer, FLASH_MAX_WORDS * sizeof(uint32_t)) != mscReturnOk) {
		LWB_SERIAL_Println("FLASH: write failed");
		MSC_Deinit();
		return false;
	}

	LWB_SERIAL_Debugln("FLASH: write successful");

	MSC_Deinit();
	isEditing = false;
	return true;
}

uint32_t LWB_SETTINGS_Read(uint8_t index) {
	if (index < 0 || index >= FLASH_MAX_WORDS) {
		LWB_SERIAL_Printfln("FLASH: Word %d is out of range", index);
		return 0.;
	}
	if (isEditing) {
		return buffer[index];
	} else {
		uint32_t *address = (uint32_t*)(FLASH_START_ADDR + index * sizeof(uint32_t));
		uint32_t data = *address;
		if (data == 0xFFFFFFFF) {
			LWB_SERIAL_Printfln("FLASH: position %d seems to be empty", index);
		}
		return data;
	}
}

float LWB_SETTINGS_ReadFloat(uint8_t index) {
	uint32_t data = LWB_SETTINGS_Read(index);
	return *(float *)&data;
}

bool LWB_SETTINGS_IsNAN(float val) {
	return *(uint32_t *)&val == 0xFFFFFFFF;
}


#endif /* LWB_PLATFORM_TD120X */
