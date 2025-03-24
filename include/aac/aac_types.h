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

#ifndef _AAC_TYPES_H_
#define _AAC_TYPES_H_


#include <audio-defs/adefs.h>


/**
 * Table 1.17 – Audio Object Types
 */
enum aac_audioObjectType {
	/* Unknown audio object type */
	AAC_AOT_NULL = 0,

	/* Main */
	AAC_AOT_AAC_MAIN,

	/* Low Complexity */
	AAC_AOT_AAC_LC,

	/* Scalable Sample Rate */
	AAC_AOT_AAC_SSR,

	/* Long Term Prediction */
	AAC_AOT_AAC_LTP,

	/* Spectral Band Replication */
	AAC_AOT_SBR,

	/* Scalable */
	AAC_AOT_AAC_SCALABLE,

	/* Twin Vector Quantizer */
	AAC_AOT_TWINVQ,

	/* Code Excited Linear Prediction */
	AAC_AOT_CELP,

	/* Harmonic Vector eXcitation Coding */
	AAC_AOT_HVXC,

	/* 10, 11: Reserved */

	/* Text-To-Speech Interface */
	AAC_AOT_TTSI = 12,

	/* Main Synthesis */
	AAC_AOT_MAINSYNTH,

	/* Wavetable Synthesis */
	AAC_AOT_WAVESYNTH,

	/* General MIDI */
	AAC_AOT_MIDI,

	/* Algorithmic Synthesis and Audio Effects */
	AAC_AOT_SAFX,

	/* Error Resilient Low Complexity */
	AAC_AOT_ER_AAC_LC,

	/* 18: Reserved */

	/* Error Resilient Long Term Prediction */
	AAC_AOT_ER_AAC_LTP = 19,

	/* Error Resilient Scalable */
	AAC_AOT_ER_AAC_SCALABLE,

	/* Error Resilient Twin Vector Quantizer */
	AAC_AOT_ER_TWINVQ,

	/* Error Resilient Bit-Sliced Arithmetic Coding */
	AAC_AOT_ER_BSAC,

	/* Error Resilient Low Delay */
	AAC_AOT_ER_AAC_LD,

	/* Error Resilient Code Excited Linear Prediction */
	AAC_AOT_ER_CELP,

	/* Error Resilient Harmonic Vector eXcitation Coding */
	AAC_AOT_ER_HVXC,

	/* Error Resilient Harmonic and Individual Lines plus Noise */
	AAC_AOT_ER_HILN,

	/* Error Resilient Parametric */
	AAC_AOT_ER_PARAM,

	/* SinuSoidal Coding */
	AAC_AOT_SSC,

	/* Parametric Stereo */
	AAC_AOT_PS,

	/* MPEG Surround */
	AAC_AOT_SURROUND,

	/* Escape Value */
	AAC_AOT_ESCAPE,

	/* Layer 1 */
	AAC_AOT_L1,

	/* Layer 2 */
	AAC_AOT_L2,

	/* Layer 3 */
	AAC_AOT_L3,

	/* Direct Stream Transfer */
	AAC_AOT_DST,

	/* Audio LosslesS */
	AAC_AOT_ALS,

	/* Scalable LosslesS */
	AAC_AOT_SLS,

	/* Scalable LosslesS (non core) */
	AAC_AOT_SLS_NON_CORE,

	/* Error Resilient Enhanced Low Delay */
	AAC_AOT_ER_AAC_ELD,

	/* Symbolic Music Representation Simple */
	AAC_AOT_SMR_SIMPLE,

	/* Symbolic Music Representation Main */
	AAC_AOT_SMR_MAIN,

	/* Enum values count (invalid value) */
	AAC_AOT_MAX,
};


/**
 * Table 4.121 – Values of the extension_type field
 */
enum aac_extension_type {
	/* Bitstream payload filler */
	AAC_EXT_TYPE_FILL = 0,

	/* Bitstream payload data as filler */
	AAC_EXT_TYPE_FILL_DATA = 0x1,

	/* Data element */
	AAC_EXT_DATA_ELEMENT = 0x2,

	/* Dynamic range control */
	AAC_EXT_DYNAMIC_RANGE = 0xB,

	/* MPEG Surround */
	AAC_EXT_SAC_DATA = 0xC,

	/* SBR enhancement */
	AAC_EXT_SBR_DATA = 0xD,

	/* SBR enhancement with CRC */
	AAC_EXT_SBR_DATA_CRC = 0xE,
};


/**
 * Table 4.85 – Syntactic elements
 */
enum aac_syntactic_element_id {
	/* Invalid */
	AAC_SYN_ELE_ID_NONE = -1,

	/* Single Channel Element */
	AAC_SYN_ELE_ID_SCE = 0x0,

	/* Channel Pair Element */
	AAC_SYN_ELE_ID_CPE = 0x1,

	/* Coupling Channel Element */
	AAC_SYN_ELE_ID_CCE = 0x2,

	/* LFE Channel Element */
	AAC_SYN_ELE_ID_LFE = 0x3,

	/* Data Stream Element */
	AAC_SYN_ELE_ID_DSE = 0x4,

	/* Program Config Element */
	AAC_SYN_ELE_ID_PCE = 0x5,

	/* Fill Element */
	AAC_SYN_ELE_ID_FIL = 0x6,

	/* End Element = Terminator */
	AAC_SYN_ELE_ID_END = 0x7,
};


/**
 * Table 4.1 – Syntax of GASpecificConfig()
 */
struct aac_GASpecificConfig {
	uint8_t frameLengthFlag; /* 0: length=1024, 1: length=960 */
	uint8_t dependsOnCoreCoder;
	uint16_t coreCoderDelay;
	uint8_t extensionFlag;

	/* Only for AOT 6, 20 */
	uint8_t layerNr;

	/* Only for AOT 22 */
	uint8_t numOfSubFrame;
	uint16_t layer_length;

	/* Only for AOT 17, 19, 20, 23 */
	uint8_t aacSectionDataResilienceFlag;
	uint8_t aacScalefactorDataResilienceFlag;
	uint8_t aacSpectralDataResilienceFlag;

	uint8_t extensionFlag3;
};


/**
 * 1.15 – Syntax of AudioSpecificConfig
 */
struct aac_asc {
	enum aac_audioObjectType audioObjectType;
	uint8_t samplingFrequencyIndex;
	uint16_t samplingFrequency;
	uint8_t channelConfiguration;
	/* Only for AOT 5, 29 */
	uint8_t extensionSamplingFrequencyIndex;
	uint16_t extensionSamplingFrequency;
	/* Only for AOT 22 */
	uint8_t extensionChannelConfiguration;
	/* Only for AOT 30 */
	uint8_t sacPayloadEmbedding;
	/* Only for AOT 36 */
	uint8_t fillBits;
	/* Only for AOT 39 */
	uint8_t epConfig;
	uint8_t directMapping;
	uint16_t syncExtensionType;
	/* Only for AOT 5, 22 */
	uint8_t sbrPresentFlag;
	uint8_t psPresentFlag;
	union {
		struct aac_GASpecificConfig GASpecificConfig;
	};
};


/**
 * 1.A.3.2.1 Fixed Header of ADTS
 */
struct aac_adts {
	/* 1.A.3.2.1 Fixed Header of ADTS */
	uint16_t syncword;
	uint8_t ID;
	uint8_t layer;
	uint8_t protection_absent;
	uint8_t profile_ObjectType; /* enum aac_audioObjectType minus 1 */
	uint8_t sampling_frequency_index;
	uint8_t private_bit;
	uint8_t channel_configuration;
	uint8_t original_copy;
	uint8_t home;
	/* 1.A.3.2.2 Variable Header of ADTS */
	uint8_t copyright_identification_bit;
	uint8_t copyright_identification_start;
	uint16_t aac_frame_length;
	uint16_t adts_buffer_fullness; /* VBR: 0x7FF */
	uint8_t number_of_raw_data_blocks_in_frame; /* minus 1 */
};


#define AAC_MAX_WINDOW_GROUPS 8
#define AAC_MAX_SFB 64
#define AAC_MAX_RAW_DATA_BLOCKS 4
#define AAC_MAX_SYN_ELE 10 /* TODO: dynamic */


/**
 * Table 4.128 – Window Sequences
 */
enum aac_window_sequence {
	ONLY_LONG_SEQUENCE = 0,
	LONG_START_SEQUENCE = 1,
	EIGHT_SHORT_SEQUENCE = 2,
	LONG_STOP_SEQUENCE = 3,
};


/**
 * 4.6.3.2 Definitions
 */
enum aac_band_type {
	ZERO_HCB = 0,
	FIRST_PAIR_HCB = 5,
	ESC_HCB = 11,
	NOISE_HCB = 13,
	INTENSITY_HCB2 = 14,
	INTENSITY_HCB = 15,
	ESC_FLAG = 16,
};


/**
 * Table 4.6 – Syntax of ics_info()
 */
struct aac_ics_info {
	uint8_t ics_reserved_bit;
	uint8_t window_sequence;
	uint8_t window_shape;
	uint8_t max_sfb;
	uint8_t scale_factor_grouping;
	uint8_t predictor_data_present;
	uint8_t predictor_reset;
	uint8_t predictor_reset_group_number;
	uint8_t prediction_used[AAC_MAX_SFB];
	uint8_t ltp_data_present;
};


/**
 * Table 4.52 – Syntax of section_data()
 */
struct aac_section_data {
	uint8_t sect_cb[AAC_MAX_WINDOW_GROUPS][AAC_MAX_SFB];
	uint16_t sect_start[AAC_MAX_WINDOW_GROUPS][AAC_MAX_SFB];
	uint16_t sect_end[AAC_MAX_WINDOW_GROUPS][AAC_MAX_SFB];
	uint8_t sfb_cb[AAC_MAX_WINDOW_GROUPS][AAC_MAX_SFB];
	uint8_t num_sec[AAC_MAX_WINDOW_GROUPS];
};


/**
 * Table 4.53 – Syntax of scale_factor_data()
 */
struct aac_scale_factor_data {
	uint8_t sf_concealment;
	uint16_t length_of_rvlc_sf;
	int16_t dpcm_is_position[AAC_MAX_WINDOW_GROUPS][AAC_MAX_SFB];
	int16_t dpcm_noise_nrg[AAC_MAX_WINDOW_GROUPS][AAC_MAX_SFB];
	int16_t dpcm_sf[AAC_MAX_WINDOW_GROUPS][AAC_MAX_SFB];
};


/**
 *  Table 4.7 – Syntax of pulse_data()
 */
struct aac_pulse_data {
	uint8_t number_pulse;
	uint8_t pulse_start_sfb;
	uint8_t pulse_offset[4];
	uint8_t pulse_amp[4];
};


/**
 * Table 4.54 – Syntax of tns_data()
 */
struct aac_tns_data {
	uint8_t n_filt[8];
	uint8_t coef_res[AAC_MAX_WINDOW_GROUPS];
	uint8_t length[AAC_MAX_WINDOW_GROUPS][4];
	uint8_t order[AAC_MAX_WINDOW_GROUPS][4];
	uint8_t direction[AAC_MAX_WINDOW_GROUPS][4];
	uint8_t coef_compress[AAC_MAX_WINDOW_GROUPS][4];
	uint8_t coef[AAC_MAX_WINDOW_GROUPS][4][32];
};


/**
 * Table 4.12 – Syntax of gain_control_data()
 */
struct aac_gain_control_data {
	uint8_t max_band;
	uint8_t adjust_num[4][8];
	uint8_t alevcode[4][8][8];
	uint8_t aloccode[4][8][8];
};


/**
 * Table 4.56 – Syntax of spectral_data()
 */
struct aac_spectral_data {
	/* TODO */
	int empty;
};


/**
 * Table 4.51 – Syntax of reordered_spectral_data ()
 */
struct aac_reordered_spectral_data {
	/* TODO */
	int empty;
};


/**
 * Table 4.50 – Syntax of individual_channel_stream()
 */
struct aac_individual_channel_stream {
	uint8_t global_gain;
	struct aac_ics_info ics_info;
	struct aac_section_data section_data;
	struct aac_scale_factor_data scale_factor_data;
	uint8_t pulse_data_present;
	struct aac_pulse_data pulse_data;
	uint8_t tns_data_present;
	struct aac_tns_data tns_data;
	uint8_t gain_control_data_present;
	struct aac_gain_control_data gain_control_data;
	uint16_t length_of_reordered_spectral_data;
	uint8_t length_of_longest_codeword;
	struct aac_spectral_data spectral_data;
	struct aac_reordered_spectral_data reordered_spectral_data;
};


/**
 * Table 4.4 – Syntax of single_channel_element()
 */
struct aac_single_channel_element {
	uint8_t element_instance_tag;
	struct aac_individual_channel_stream ics;
};


/**
 * Table 4.5 – Syntax of channel_pair_element()
 */
struct aac_channel_pair_element {
	uint8_t element_instance_tag;
	uint8_t common_window;
	struct aac_ics_info ics_info;
	uint8_t ms_mask_present;
	uint8_t ms_used[AAC_MAX_WINDOW_GROUPS][AAC_MAX_SFB];
	struct aac_individual_channel_stream ics1;
	struct aac_individual_channel_stream ics2;
};


/**
 * Table 4.8 – Syntax of coupling_channel_element()
 */
struct aac_coupling_channel_element {
	uint8_t element_instance_tag;
	uint8_t ind_sw_cce_flag;
	uint8_t num_coupled_element;
	uint8_t cc_target_is_cpe[8];
	uint8_t cc_target_tag_select[8];
	uint8_t cc_l[8];
	uint8_t cc_r[8];
	uint8_t cc_domain;
	uint8_t gain_element_sign;
	uint8_t gain_element_scale;
	uint8_t common_gain_element_present[8];
	uint8_t common_gain_element[8];
	uint8_t dpcm_gain_element[8][AAC_MAX_WINDOW_GROUPS][AAC_MAX_SFB];
	struct aac_individual_channel_stream ics;
};


/**
 * Table 4.10 – Syntax of data_stream_element()
 */
struct aac_data_stream_element {
	uint8_t element_instance_tag;
	uint8_t data_byte_align_flag;
	uint8_t count;
	uint8_t esc_count;
};


/**
 * Table 4.2 – Syntax of program_config_element()
 */
struct aac_program_config_element {
	uint8_t element_instance_tag;
	uint8_t object_type;
	uint8_t sampling_frequency_index;
	uint8_t num_front_channel_elements;
	uint8_t num_side_channel_elements;

	uint8_t num_back_channel_elements;
	uint8_t num_lfe_channel_elements;
	uint8_t num_assoc_data_elements;
	uint8_t num_valid_cc_elements;
	uint8_t mono_mixdown_present;
	uint8_t mono_mixdown_element_number;
	uint8_t stereo_mixdown_present;
	uint8_t stereo_mixdown_element_number;

	uint8_t matrix_mixdown_idx_present;
	uint8_t matrix_mixdown_idx;
	uint8_t pseudo_surround_enable;

	uint8_t front_element_is_cpe[16];
	uint8_t front_element_tag_select[16];

	uint8_t side_element_is_cpe[16];
	uint8_t side_element_tag_select[16];

	uint8_t back_element_is_cpe[16];
	uint8_t back_element_tag_select[16];

	uint8_t lfe_element_tag_select[4];
	uint8_t assoc_data_element_tag_select[8];
	uint8_t cc_element_is_ind_sw[16];
	uint8_t valid_cc_element_tag_select[16];

	uint8_t comment_field_bytes;
	uint8_t comment_field_data[255];
};


/**
 * Table 4.57 – Syntax of extension_payload()
 */
struct aac_extension_payload {
	enum aac_extension_type extension_type;
};


/**
 * Table 4.11 – Syntax of fill_element()
 */
struct aac_fill_element {
	uint16_t count;
	struct aac_extension_payload extension_payload;
};


struct aac_syntactic_element {
	enum aac_syntactic_element_id id_syn_ele;
	union {
		struct aac_single_channel_element sce;
		struct aac_channel_pair_element cpe;
		struct aac_coupling_channel_element cce;
		struct aac_data_stream_element dse;
		struct aac_program_config_element pce;
		struct aac_fill_element fil;
	};
};


/**
 *  Table 4.3 – Syntax of top level payload for audio object types AAC Main,
 * SSR, LC, and LTP
 */
struct aac_raw_data_block {
	struct aac_syntactic_element elements[AAC_MAX_SYN_ELE];
	size_t elements_count;
};


/**
 * Table 1.A.8 – Syntax of adts_error_check
 */
struct aac_adts_error_check {
	uint16_t crc_check;
};


/**
 * Table 1.A.9 – Syntax of adts_header_error_check
 */
struct aac_adts_header_error_check {
	uint16_t raw_data_block_position[AAC_MAX_RAW_DATA_BLOCKS];
	uint16_t crc_check;
};


/**
 * Table 1.A.10 – Syntax of adts_raw_data_block_error_check
 */
struct aac_adts_raw_data_block_error_check {
	uint16_t crc_check;
};


/**
 * Table 1.A.5 – Syntax of adts_frame()
 */
struct aac_adts_frame {
	struct aac_adts adts;
	union {
		struct aac_adts_error_check adts_error_check;
		struct aac_adts_header_error_check adts_header_error_check;
	};
	struct aac_raw_data_block raw_data_block[AAC_MAX_RAW_DATA_BLOCKS];
	struct aac_adts_raw_data_block_error_check
		adts_raw_data_block_error_check[AAC_MAX_RAW_DATA_BLOCKS];
};


/**
 * Scalefactor bands and grouping
 */
struct aac_scalefactor_bands_and_grouping {
	uint8_t num_windows;
	uint8_t num_window_groups;
	uint8_t window_group_length[AAC_MAX_WINDOW_GROUPS];
	uint8_t num_swb;
	uint16_t sect_sfb_offset[AAC_MAX_WINDOW_GROUPS][AAC_MAX_SFB];
	uint16_t swb_offset[AAC_MAX_SFB];
};


/**
 * Get an enum aac_audioObjectType value from a string.
 * Valid strings are only the suffix of the audio object type name (eg.
 * 'AAC_AOT_AAC_LC'). The case is ignored.
 * @param str: audio objet type name to convert
 * @return the enum aac_audioObjectType value or AAC_AOT_NULL if unknown
 */
AAC_API enum aac_audioObjectType aac_aot_from_str(const char *str);


/**
 * Get a string from an enum aac_audioObjectType value.
 * @param aot: audio object type value to convert
 * @return a string description of the audio object typ
 */
AAC_API const char *aac_aot_to_str(enum aac_audioObjectType aot);


#endif /* !_AAC_TYPES_H_ */
