/**
 * Copyright (c) 2023 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "aac_test.h"


static void test_aot_from_str(void)
{
	enum aac_audioObjectType value;

	value = aac_aot_from_str(NULL);
	CU_ASSERT_EQUAL(value, AAC_AOT_NULL);

	value = aac_aot_from_str("?");
	CU_ASSERT_EQUAL(value, AAC_AOT_NULL);

	value = aac_aot_from_str("abcde");
	CU_ASSERT_EQUAL(value, AAC_AOT_NULL);

	value = aac_aot_from_str("NULL");
	CU_ASSERT_EQUAL(value, AAC_AOT_NULL);

	value = aac_aot_from_str("AAC_LC");
	CU_ASSERT_EQUAL(value, AAC_AOT_AAC_LC);
}


static void test_aot_to_str(void)
{
	const char *value;

	value = aac_aot_to_str(-1);
	CU_ASSERT_STRING_EQUAL(value, "UNKNOWN");

	value = aac_aot_to_str(AAC_AOT_NULL);
	CU_ASSERT_STRING_EQUAL(value, "NULL");

	value = aac_aot_to_str(AAC_AOT_AAC_LC);
	CU_ASSERT_STRING_EQUAL(value, "AAC_LC");

	value = aac_aot_to_str(AAC_AOT_MAX);
	CU_ASSERT_STRING_EQUAL(value, "UNKNOWN");

	value = aac_aot_to_str(AAC_AOT_MAX + 1);
	CU_ASSERT_STRING_EQUAL(value, "UNKNOWN");
}


CU_TestInfo g_aac_test_str[] = {
	{FN("aot-from-str"), &test_aot_from_str},
	{FN("aot-to-str"), &test_aot_to_str},

	CU_TEST_INFO_NULL,
};
