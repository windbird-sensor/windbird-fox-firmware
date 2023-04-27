/*
 * lwb_utils.c
 *
 *  Created on: 11 Apr 2023
 *      Author: windbird-dev
 */



/***************************************************************************//**
* @brief
 *   Convert a string to long long integer and ignore up to 1 char.
 *
 * @param[in] instr
 *  Pointer to the input string.
 *
 * @param[in] ignore
 *  Char to ignore within the String. 0 if not used.
 *
 * @return
 *  The converted long long integer value.
 ******************************************************************************/
long long LWB_Utils_atolli(char *instr, char ignore)
{
	long long retval = 0, sign = 1;

	// Skip leading space and tab characters
	for (; *instr == ' ' || *instr == '\t'; instr++) {
		;
	}

	// Handle optional sign
	if (*instr == '-') {
		sign = -1;
		instr++;
	}
	if (*instr == '+') {
		instr++;
	}
	for (; *instr && ((*instr >= '0' && *instr <= '9') || (*instr == ignore));
		instr++) {
		if (*instr != ignore) {
			retval = (10 * retval) + (*instr - '0');
		}
	}
	return sign * retval;
}
