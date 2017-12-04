/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2012-2013 Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef __NVME_H__
#define __NVME_H__

#ifdef _KERNEL
#include <sys/types.h>
#endif

#include <sys/param.h>
#include <sys/endian.h>

#define	NVME_PASSTHROUGH_CMD		_IOWR('n', 0, struct nvme_pt_command)
#define	NVME_RESET_CONTROLLER		_IO('n', 1)

#define	NVME_IO_TEST			_IOWR('n', 100, struct nvme_io_test)
#define	NVME_BIO_TEST			_IOWR('n', 101, struct nvme_io_test)

/*
 * Macros to deal with NVME revisions, as defined VS register
 */
#define NVME_REV(x, y)			(((x) << 16) | ((y) << 8))
#define NVME_MAJOR(r)			(((r) >> 16) & 0xffff)
#define NVME_MINOR(r)			(((r) >> 8) & 0xff)

/*
 * Use to mark a command to apply to all namespaces, or to retrieve global
 *  log pages.
 */
#define NVME_GLOBAL_NAMESPACE_TAG	((uint32_t)0xFFFFFFFF)

/* Cap nvme to 1MB transfers driver explodes with larger sizes */
#define NVME_MAX_XFER_SIZE		(MAXPHYS < (1<<20) ? MAXPHYS : (1<<20))

/* Register field definitions */
#define NVME_CAP_LO_REG_MQES_SHIFT			(0)
#define NVME_CAP_LO_REG_MQES_MASK			(0xFFFF)
#define NVME_CAP_LO_REG_CQR_SHIFT			(16)
#define NVME_CAP_LO_REG_CQR_MASK			(0x1)
#define NVME_CAP_LO_REG_AMS_SHIFT			(17)
#define NVME_CAP_LO_REG_AMS_MASK			(0x3)
#define NVME_CAP_LO_REG_TO_SHIFT			(24)
#define NVME_CAP_LO_REG_TO_MASK				(0xFF)

#define NVME_CAP_HI_REG_DSTRD_SHIFT			(0)
#define NVME_CAP_HI_REG_DSTRD_MASK			(0xF)
#define NVME_CAP_HI_REG_CSS_NVM_SHIFT			(5)
#define NVME_CAP_HI_REG_CSS_NVM_MASK			(0x1)
#define NVME_CAP_HI_REG_MPSMIN_SHIFT			(16)
#define NVME_CAP_HI_REG_MPSMIN_MASK			(0xF)
#define NVME_CAP_HI_REG_MPSMAX_SHIFT			(20)
#define NVME_CAP_HI_REG_MPSMAX_MASK			(0xF)

#define NVME_CC_REG_EN_SHIFT				(0)
#define NVME_CC_REG_EN_MASK				(0x1)
#define NVME_CC_REG_CSS_SHIFT				(4)
#define NVME_CC_REG_CSS_MASK				(0x7)
#define NVME_CC_REG_MPS_SHIFT				(7)
#define NVME_CC_REG_MPS_MASK				(0xF)
#define NVME_CC_REG_AMS_SHIFT				(11)
#define NVME_CC_REG_AMS_MASK				(0x7)
#define NVME_CC_REG_SHN_SHIFT				(14)
#define NVME_CC_REG_SHN_MASK				(0x3)
#define NVME_CC_REG_IOSQES_SHIFT			(16)
#define NVME_CC_REG_IOSQES_MASK				(0xF)
#define NVME_CC_REG_IOCQES_SHIFT			(20)
#define NVME_CC_REG_IOCQES_MASK				(0xF)

#define NVME_CSTS_REG_RDY_SHIFT				(0)
#define NVME_CSTS_REG_RDY_MASK				(0x1)
#define NVME_CSTS_REG_CFS_SHIFT				(1)
#define NVME_CSTS_REG_CFS_MASK				(0x1)
#define NVME_CSTS_REG_SHST_SHIFT			(2)
#define NVME_CSTS_REG_SHST_MASK				(0x3)

#define NVME_CSTS_GET_SHST(csts)			(((csts) >> NVME_CSTS_REG_SHST_SHIFT) & NVME_CSTS_REG_SHST_MASK)

#define NVME_AQA_REG_ASQS_SHIFT				(0)
#define NVME_AQA_REG_ASQS_MASK				(0xFFF)
#define NVME_AQA_REG_ACQS_SHIFT				(16)
#define NVME_AQA_REG_ACQS_MASK				(0xFFF)

/* Command field definitions */

#define NVME_CMD_OPC_SHIFT				(0)
#define NVME_CMD_OPC_MASK				(0xFF)
#define NVME_CMD_FUSE_SHIFT				(8)
#define NVME_CMD_FUSE_MASK				(0x3)

#define NVME_CMD_SET_OPC(opc)				(((opc) & NVME_CMD_OPC_MASK) << NVME_CMD_OPC_SHIFT)

#define NVME_STATUS_P_SHIFT				(0)
#define NVME_STATUS_P_MASK				(0x1)
#define NVME_STATUS_SC_SHIFT				(1)
#define NVME_STATUS_SC_MASK				(0xFF)
#define NVME_STATUS_SCT_SHIFT				(9)
#define NVME_STATUS_SCT_MASK				(0x7)
#define NVME_STATUS_M_SHIFT				(14)
#define NVME_STATUS_M_MASK				(0x1)
#define NVME_STATUS_DNR_SHIFT				(15)
#define NVME_STATUS_DNR_MASK				(0x1)

#define NVME_STATUS_GET_P(st)				(((st) >> NVME_STATUS_P_SHIFT) & NVME_STATUS_P_MASK)
#define NVME_STATUS_GET_SC(st)				(((st) >> NVME_STATUS_SC_SHIFT) & NVME_STATUS_SC_MASK)
#define NVME_STATUS_GET_SCT(st)				(((st) >> NVME_STATUS_SCT_SHIFT) & NVME_STATUS_SCT_MASK)
#define NVME_STATUS_GET_M(st)				(((st) >> NVME_STATUS_M_SHIFT) & NVME_STATUS_M_MASK)
#define NVME_STATUS_GET_DNR(st)				(((st) >> NVME_STATUS_DNR_SHIFT) & NVME_STATUS_DNR_MASK)

#define NVME_PWR_ST_MPS_SHIFT				(0)
#define NVME_PWR_ST_MPS_MASK				(0x1)
#define NVME_PWR_ST_NOPS_SHIFT				(1)
#define NVME_PWR_ST_NOPS_MASK				(0x1)
#define NVME_PWR_ST_RRT_SHIFT				(0)
#define NVME_PWR_ST_RRT_MASK				(0x1F)
#define NVME_PWR_ST_RRL_SHIFT				(0)
#define NVME_PWR_ST_RRL_MASK				(0x1F)
#define NVME_PWR_ST_RWT_SHIFT				(0)
#define NVME_PWR_ST_RWT_MASK				(0x1F)
#define NVME_PWR_ST_RWL_SHIFT				(0)
#define NVME_PWR_ST_RWL_MASK				(0x1F)
#define NVME_PWR_ST_IPS_SHIFT				(6)
#define NVME_PWR_ST_IPS_MASK				(0x3)
#define NVME_PWR_ST_APW_SHIFT				(0)
#define NVME_PWR_ST_APW_MASK				(0x7)
#define NVME_PWR_ST_APS_SHIFT				(6)
#define NVME_PWR_ST_APS_MASK				(0x3)

#define NVME_CTRLR_DATA_OACS_SECURITY_SHIFT		(0)
#define NVME_CTRLR_DATA_OACS_SECURITY_MASK		(0x1)
#define NVME_CTRLR_DATA_OACS_FORMAT_SHIFT		(1)
#define NVME_CTRLR_DATA_OACS_FORMAT_MASK		(0x1)
#define NVME_CTRLR_DATA_OACS_FIRMWARE_SHIFT		(2)
#define NVME_CTRLR_DATA_OACS_FIRMWARE_MASK		(0x1)
#define NVME_CTRLR_DATA_OACS_NSMGMT_SHIFT		(3)
#define NVME_CTRLR_DATA_OACS_NSMGMT_MASK		(0x1)
#define NVME_CTRLR_DATA_FRMW_SLOT1_RO_SHIFT		(0)
#define NVME_CTRLR_DATA_FRMW_SLOT1_RO_MASK		(0x1)
#define NVME_CTRLR_DATA_FRMW_NUM_SLOTS_SHIFT		(1)
#define NVME_CTRLR_DATA_FRMW_NUM_SLOTS_MASK		(0x7)
#define NVME_CTRLR_DATA_LPA_NS_SMART_SHIFT		(0)
#define NVME_CTRLR_DATA_LPA_NS_SMART_MASK		(0x1)
#define NVME_CTRLR_DATA_AVSCC_SPEC_FORMAT_SHIFT		(0)
#define NVME_CTRLR_DATA_AVSCC_SPEC_FORMAT_MASK		(0x1)
#define NVME_CTRLR_DATA_APSTA_APST_SUPP_SHIFT		(0)
#define NVME_CTRLR_DATA_APSTA_APST_SUPP_MASK		(0x1)
#define NVME_CTRLR_DATA_SQES_MIN_SHIFT			(0)
#define NVME_CTRLR_DATA_SQES_MIN_MASK			(0xF)
#define NVME_CTRLR_DATA_SQES_MAX_SHIFT			(4)
#define NVME_CTRLR_DATA_SQES_MAX_MASK			(0xF)
#define NVME_CTRLR_DATA_CQES_MIN_SHIFT			(0)
#define NVME_CTRLR_DATA_CQES_MIN_MASK			(0xF)
#define NVME_CTRLR_DATA_CQES_MAX_SHIFT			(4)
#define NVME_CTRLR_DATA_CQES_MAX_MASK			(0xF)
#define NVME_CTRLR_DATA_ONCS_COMPARE_SHIFT		(0)
#define NVME_CTRLR_DATA_ONCS_COMPARE_MASK		(0x1)
#define NVME_CTRLR_DATA_ONCS_WRITE_UNC_SHIFT		(1)
#define NVME_CTRLR_DATA_ONCS_WRITE_UNC_MASK		(0x1)
#define NVME_CTRLR_DATA_ONCS_DSM_SHIFT			(2)
#define NVME_CTRLR_DATA_ONCS_DSM_MASK			(0x1)
#define NVME_CTRLR_DATA_VWC_PRESENT_SHIFT		(0)
#define NVME_CTRLR_DATA_VWC_PRESENT_MASK		(0x1)

#define NVME_NS_DATA_NSFEAT_THIN_PROV_SHIFT		(0)
#define NVME_NS_DATA_NSFEAT_THIN_PROV_MASK		(0x1)
#define NVME_NS_DATA_FLBAS_FORMAT_SHIFT			(0)
#define NVME_NS_DATA_FLBAS_FORMAT_MASK			(0xF)
#define NVME_NS_DATA_FLBAS_EXTENDED_SHIFT		(4)
#define NVME_NS_DATA_FLBAS_EXTENDED_MASK		(0x1)
#define NVME_NS_DATA_MC_EXTENDED_SHIFT			(0)
#define NVME_NS_DATA_MC_EXTENDED_MASK			(0x1)
#define NVME_NS_DATA_MC_POINTER_SHIFT			(1)
#define NVME_NS_DATA_MC_POINTER_MASK			(0x1)
#define NVME_NS_DATA_DPC_PIT1_SHIFT			(0)
#define NVME_NS_DATA_DPC_PIT1_MASK			(0x1)
#define NVME_NS_DATA_DPC_PIT2_SHIFT			(1)
#define NVME_NS_DATA_DPC_PIT2_MASK			(0x1)
#define NVME_NS_DATA_DPC_PIT3_SHIFT			(2)
#define NVME_NS_DATA_DPC_PIT3_MASK			(0x1)
#define NVME_NS_DATA_DPC_MD_START_SHIFT			(3)
#define NVME_NS_DATA_DPC_MD_START_MASK			(0x1)
#define NVME_NS_DATA_DPC_MD_END_SHIFT			(4)
#define NVME_NS_DATA_DPC_MD_END_MASK			(0x1)
#define NVME_NS_DATA_DPS_PIT_SHIFT			(0)
#define NVME_NS_DATA_DPS_PIT_MASK			(0x7)
#define NVME_NS_DATA_DPS_MD_START_SHIFT			(3)
#define NVME_NS_DATA_DPS_MD_START_MASK			(0x1)
#define NVME_NS_DATA_LBAF_MS_SHIFT			(0)
#define NVME_NS_DATA_LBAF_MS_MASK			(0xFFFF)
#define NVME_NS_DATA_LBAF_LBADS_SHIFT			(16)
#define NVME_NS_DATA_LBAF_LBADS_MASK			(0xFF)
#define NVME_NS_DATA_LBAF_RP_SHIFT			(24)
#define NVME_NS_DATA_LBAF_RP_MASK			(0x3)

#define NVME_CRIT_WARN_ST_AVAILABLE_SPARE_SHIFT		(0)
#define NVME_CRIT_WARN_ST_AVAILABLE_SPARE_MASK		(0x1)
#define NVME_CRIT_WARN_ST_TEMPERATURE_SHIFT		(1)
#define NVME_CRIT_WARN_ST_TEMPERATURE_MASK		(0x1)
#define NVME_CRIT_WARN_ST_DEVICE_RELIABILITY_SHIFT	(2)
#define NVME_CRIT_WARN_ST_DEVICE_RELIABILITY_MASK	(0x1)
#define NVME_CRIT_WARN_ST_READ_ONLY_SHIFT		(3)
#define NVME_CRIT_WARN_ST_READ_ONLY_MASK		(0x1)
#define NVME_CRIT_WARN_ST_VOLATILE_MEMORY_BACKUP_SHIFT	(4)
#define NVME_CRIT_WARN_ST_VOLATILE_MEMORY_BACKUP_MASK	(0x1)

#define NVME_FIRMWARE_PAGE_AFI_SLOT_SHIFT		(0)
#define NVME_FIRMWARE_PAGE_AFI_SLOT_MASK		(0x7)

/* CC register SHN field values */
enum shn_value {
	NVME_SHN_NORMAL		= 0x1,
	NVME_SHN_ABRUPT		= 0x2,
};

/* CSTS register SHST field values */
enum shst_value {
	NVME_SHST_NORMAL	= 0x0,
	NVME_SHST_OCCURRING	= 0x1,
	NVME_SHST_COMPLETE	= 0x2,
};

struct nvme_registers
{
	/** controller capabilities */
	uint32_t		cap_lo;
	uint32_t		cap_hi;

	uint32_t		vs;	/* version */
	uint32_t		intms;	/* interrupt mask set */
	uint32_t		intmc;	/* interrupt mask clear */

	/** controller configuration */
	uint32_t		cc;

	uint32_t		reserved1;

	/** controller status */
	uint32_t		csts;

	uint32_t		reserved2;

	/** admin queue attributes */
	uint32_t		aqa;

	uint64_t		asq;	/* admin submission queue base addr */
	uint64_t		acq;	/* admin completion queue base addr */
	uint32_t		reserved3[0x3f2];

	struct {
	    uint32_t		sq_tdbl; /* submission queue tail doorbell */
	    uint32_t		cq_hdbl; /* completion queue head doorbell */
	} doorbell[1] __packed;
} __packed;

_Static_assert(sizeof(struct nvme_registers) == 0x1008, "bad size for nvme_registers");

struct nvme_command
{
	/* dword 0 */
	uint16_t opc_fuse;	/* opcode, fused operation */
	uint16_t cid;		/* command identifier */

	/* dword 1 */
	uint32_t nsid;		/* namespace identifier */

	/* dword 2-3 */
	uint32_t rsvd2;
	uint32_t rsvd3;

	/* dword 4-5 */
	uint64_t mptr;		/* metadata pointer */

	/* dword 6-7 */
	uint64_t prp1;		/* prp entry 1 */

	/* dword 8-9 */
	uint64_t prp2;		/* prp entry 2 */

	/* dword 10-15 */
	uint32_t cdw10;		/* command-specific */
	uint32_t cdw11;		/* command-specific */
	uint32_t cdw12;		/* command-specific */
	uint32_t cdw13;		/* command-specific */
	uint32_t cdw14;		/* command-specific */
	uint32_t cdw15;		/* command-specific */
} __packed;

_Static_assert(sizeof(struct nvme_command) == 16 * 4, "bad size for nvme_command");

struct nvme_completion {

	/* dword 0 */
	uint32_t		cdw0;	/* command-specific */

	/* dword 1 */
	uint32_t		rsvd1;

	/* dword 2 */
	uint16_t		sqhd;	/* submission queue head pointer */
	uint16_t		sqid;	/* submission queue identifier */

	/* dword 3 */
	uint16_t		cid;	/* command identifier */
	uint16_t		status;
} __packed;

_Static_assert(sizeof(struct nvme_completion) == 4 * 4, "bad size for nvme_completion");

struct nvme_dsm_range {

	uint32_t attributes;
	uint32_t length;
	uint64_t starting_lba;
} __packed;

_Static_assert(sizeof(struct nvme_dsm_range) == 16, "bad size for nvme_dsm_ranage");

/* status code types */
enum nvme_status_code_type {
	NVME_SCT_GENERIC		= 0x0,
	NVME_SCT_COMMAND_SPECIFIC	= 0x1,
	NVME_SCT_MEDIA_ERROR		= 0x2,
	/* 0x3-0x6 - reserved */
	NVME_SCT_VENDOR_SPECIFIC	= 0x7,
};

/* generic command status codes */
enum nvme_generic_command_status_code {
	NVME_SC_SUCCESS				= 0x00,
	NVME_SC_INVALID_OPCODE			= 0x01,
	NVME_SC_INVALID_FIELD			= 0x02,
	NVME_SC_COMMAND_ID_CONFLICT		= 0x03,
	NVME_SC_DATA_TRANSFER_ERROR		= 0x04,
	NVME_SC_ABORTED_POWER_LOSS		= 0x05,
	NVME_SC_INTERNAL_DEVICE_ERROR		= 0x06,
	NVME_SC_ABORTED_BY_REQUEST		= 0x07,
	NVME_SC_ABORTED_SQ_DELETION		= 0x08,
	NVME_SC_ABORTED_FAILED_FUSED		= 0x09,
	NVME_SC_ABORTED_MISSING_FUSED		= 0x0a,
	NVME_SC_INVALID_NAMESPACE_OR_FORMAT	= 0x0b,
	NVME_SC_COMMAND_SEQUENCE_ERROR		= 0x0c,

	NVME_SC_LBA_OUT_OF_RANGE		= 0x80,
	NVME_SC_CAPACITY_EXCEEDED		= 0x81,
	NVME_SC_NAMESPACE_NOT_READY		= 0x82,
};

/* command specific status codes */
enum nvme_command_specific_status_code {
	NVME_SC_COMPLETION_QUEUE_INVALID	= 0x00,
	NVME_SC_INVALID_QUEUE_IDENTIFIER	= 0x01,
	NVME_SC_MAXIMUM_QUEUE_SIZE_EXCEEDED	= 0x02,
	NVME_SC_ABORT_COMMAND_LIMIT_EXCEEDED	= 0x03,
	/* 0x04 - reserved */
	NVME_SC_ASYNC_EVENT_REQUEST_LIMIT_EXCEEDED = 0x05,
	NVME_SC_INVALID_FIRMWARE_SLOT		= 0x06,
	NVME_SC_INVALID_FIRMWARE_IMAGE		= 0x07,
	NVME_SC_INVALID_INTERRUPT_VECTOR	= 0x08,
	NVME_SC_INVALID_LOG_PAGE		= 0x09,
	NVME_SC_INVALID_FORMAT			= 0x0a,
	NVME_SC_FIRMWARE_REQUIRES_RESET		= 0x0b,

	NVME_SC_CONFLICTING_ATTRIBUTES		= 0x80,
	NVME_SC_INVALID_PROTECTION_INFO		= 0x81,
	NVME_SC_ATTEMPTED_WRITE_TO_RO_PAGE	= 0x82,
};

/* media error status codes */
enum nvme_media_error_status_code {
	NVME_SC_WRITE_FAULTS			= 0x80,
	NVME_SC_UNRECOVERED_READ_ERROR		= 0x81,
	NVME_SC_GUARD_CHECK_ERROR		= 0x82,
	NVME_SC_APPLICATION_TAG_CHECK_ERROR	= 0x83,
	NVME_SC_REFERENCE_TAG_CHECK_ERROR	= 0x84,
	NVME_SC_COMPARE_FAILURE			= 0x85,
	NVME_SC_ACCESS_DENIED			= 0x86,
};

/* admin opcodes */
enum nvme_admin_opcode {
	NVME_OPC_DELETE_IO_SQ			= 0x00,
	NVME_OPC_CREATE_IO_SQ			= 0x01,
	NVME_OPC_GET_LOG_PAGE			= 0x02,
	/* 0x03 - reserved */
	NVME_OPC_DELETE_IO_CQ			= 0x04,
	NVME_OPC_CREATE_IO_CQ			= 0x05,
	NVME_OPC_IDENTIFY			= 0x06,
	/* 0x07 - reserved */
	NVME_OPC_ABORT				= 0x08,
	NVME_OPC_SET_FEATURES			= 0x09,
	NVME_OPC_GET_FEATURES			= 0x0a,
	/* 0x0b - reserved */
	NVME_OPC_ASYNC_EVENT_REQUEST		= 0x0c,
	NVME_OPC_NAMESPACE_MANAGEMENT		= 0x0d,
	/* 0x0e-0x0f - reserved */
	NVME_OPC_FIRMWARE_ACTIVATE		= 0x10,
	NVME_OPC_FIRMWARE_IMAGE_DOWNLOAD	= 0x11,
	NVME_OPC_NAMESPACE_ATTACHMENT		= 0x15,

	NVME_OPC_FORMAT_NVM			= 0x80,
	NVME_OPC_SECURITY_SEND			= 0x81,
	NVME_OPC_SECURITY_RECEIVE		= 0x82,
};

/* nvme nvm opcodes */
enum nvme_nvm_opcode {
	NVME_OPC_FLUSH				= 0x00,
	NVME_OPC_WRITE				= 0x01,
	NVME_OPC_READ				= 0x02,
	/* 0x03 - reserved */
	NVME_OPC_WRITE_UNCORRECTABLE		= 0x04,
	NVME_OPC_COMPARE			= 0x05,
	/* 0x06-0x07 - reserved */
	NVME_OPC_DATASET_MANAGEMENT		= 0x09,
};

enum nvme_feature {
	/* 0x00 - reserved */
	NVME_FEAT_ARBITRATION			= 0x01,
	NVME_FEAT_POWER_MANAGEMENT		= 0x02,
	NVME_FEAT_LBA_RANGE_TYPE		= 0x03,
	NVME_FEAT_TEMPERATURE_THRESHOLD		= 0x04,
	NVME_FEAT_ERROR_RECOVERY		= 0x05,
	NVME_FEAT_VOLATILE_WRITE_CACHE		= 0x06,
	NVME_FEAT_NUMBER_OF_QUEUES		= 0x07,
	NVME_FEAT_INTERRUPT_COALESCING		= 0x08,
	NVME_FEAT_INTERRUPT_VECTOR_CONFIGURATION = 0x09,
	NVME_FEAT_WRITE_ATOMICITY		= 0x0A,
	NVME_FEAT_ASYNC_EVENT_CONFIGURATION	= 0x0B,
	NVME_FEAT_AUTONOMOUS_POWER_STATE_TRANSITION = 0x0C,
	NVME_FEAT_HOST_MEMORY_BUFFER		= 0x0D,
	NVME_FEAT_TIMESTAMP			= 0x0E,
	NVME_FEAT_KEEP_ALIVE_TIMER		= 0x0F,
	NVME_FEAT_HOST_CONTROLLED_THERMAL_MGMT	= 0x10,
	NVME_FEAT_NON_OP_POWER_STATE_CONFIG	= 0x11,
	/* 0x12-0x77 - reserved */
	/* 0x78-0x7f - NVMe Management Interface */
	NVME_FEAT_SOFTWARE_PROGRESS_MARKER	= 0x80,
	/* 0x81-0xBF - command set specific (reserved) */
	/* 0xC0-0xFF - vendor specific */
};

enum nvme_dsm_attribute {
	NVME_DSM_ATTR_INTEGRAL_READ		= 0x1,
	NVME_DSM_ATTR_INTEGRAL_WRITE		= 0x2,
	NVME_DSM_ATTR_DEALLOCATE		= 0x4,
};

enum nvme_activate_action {
	NVME_AA_REPLACE_NO_ACTIVATE		= 0x0,
	NVME_AA_REPLACE_ACTIVATE		= 0x1,
	NVME_AA_ACTIVATE			= 0x2,
};

struct nvme_power_state {
	/** Maximum Power */
	uint16_t	mp;			/* Maximum Power */
	uint8_t		ps_rsvd1;
	uint8_t		mps_nops;		/* Max Power Scale, Non-Operational State */

	uint32_t	enlat;			/* Entry Latency */
	uint32_t	exlat;			/* Exit Latency */

	uint8_t		rrt;			/* Relative Read Throughput */
	uint8_t		rrl;			/* Relative Read Latency */
	uint8_t		rwt;			/* Relative Write Throughput */
	uint8_t		rwl;			/* Relative Write Latency */

	uint16_t	idlp;			/* Idle Power */
	uint8_t		ips;			/* Idle Power Scale */
	uint8_t		ps_rsvd8;

	uint16_t	actp;			/* Active Power */
	uint8_t		apw_aps;		/* Active Power Workload, Active Power Scale */
	uint8_t		ps_rsvd10[9];
} __packed;

_Static_assert(sizeof(struct nvme_power_state) == 32, "bad size for nvme_power_state");

#define NVME_SERIAL_NUMBER_LENGTH	20
#define NVME_MODEL_NUMBER_LENGTH	40
#define NVME_FIRMWARE_REVISION_LENGTH	8

struct nvme_controller_data {

	/* bytes 0-255: controller capabilities and features */

	/** pci vendor id */
	uint16_t		vid;

	/** pci subsystem vendor id */
	uint16_t		ssvid;

	/** serial number */
	uint8_t			sn[NVME_SERIAL_NUMBER_LENGTH];

	/** model number */
	uint8_t			mn[NVME_MODEL_NUMBER_LENGTH];

	/** firmware revision */
	uint8_t			fr[NVME_FIRMWARE_REVISION_LENGTH];

	/** recommended arbitration burst */
	uint8_t			rab;

	/** ieee oui identifier */
	uint8_t			ieee[3];

	/** multi-interface capabilities */
	uint8_t			mic;

	/** maximum data transfer size */
	uint8_t			mdts;

	/** Controller ID */
	uint16_t		ctrlr_id;

	/** Version */
	uint32_t		ver;

	/** RTD3 Resume Latency */
	uint32_t		rtd3r;

	/** RTD3 Enter Latency */
	uint32_t		rtd3e;

	/** Optional Asynchronous Events Supported */
	uint32_t		oaes;	/* bitfield really */

	/** Controller Attributes */
	uint32_t		ctratt;	/* bitfield really */

	uint8_t			reserved1[12];

	/** FRU Globally Unique Identifier */
	uint8_t			fguid[16];

	uint8_t			reserved2[128];

	/* bytes 256-511: admin command set attributes */

	/** optional admin command support */
	struct {
		/* supports security send/receive commands */
		uint16_t	security  : 1;

		/* supports format nvm command */
		uint16_t	format    : 1;

		/* supports firmware activate/download commands */
		uint16_t	firmware  : 1;

		/* supports namespace management commands */
		uint16_t	nsmgmt	  : 1;

		uint16_t	oacs_rsvd : 12;
	} __packed oacs;

	/** abort command limit */
	uint8_t			acl;

	/** asynchronous event request limit */
	uint8_t			aerl;

	/** firmware updates */
	struct {
		/* first slot is read-only */
		uint8_t		slot1_ro  : 1;

		/* number of firmware slots */
		uint8_t		num_slots : 3;

		uint8_t		frmw_rsvd : 4;
	} __packed frmw;

	/** log page attributes */
	struct {
		/* per namespace smart/health log page */
		uint8_t		ns_smart : 1;

		uint8_t		lpa_rsvd : 7;
	} __packed lpa;

	/** error log page entries */
	uint8_t			elpe;

	/** number of power states supported */
	uint8_t			npss;

	/** admin vendor specific command configuration */
	struct {
		/* admin vendor specific commands use spec format */
		uint8_t		spec_format : 1;

		uint8_t		avscc_rsvd  : 7;
	} __packed avscc;

	/** Autonomous Power State Transition Attributes */
	struct {
		/* Autonmous Power State Transitions supported */
		uint8_t		apst_supp : 1;

		uint8_t		apsta_rsvd : 7;
	} __packed apsta;

	/** Warning Composite Temperature Threshold */
	uint16_t		wctemp;

	/** Critical Composite Temperature Threshold */
	uint16_t		cctemp;

	/** Maximum Time for Firmware Activation */
	uint16_t		mtfa;

	/** Host Memory Buffer Preferred Size */
	uint32_t		hmpre;

	/** Host Memory Buffer Minimum Size */
	uint32_t		hmmin;

	/** Name space capabilities  */
	struct {
		/* if nsmgmt, report tnvmcap and unvmcap */
		uint8_t    tnvmcap[16];
		uint8_t    unvmcap[16];
	} __packed untncap;

	/** Replay Protected Memory Block Support */
	uint32_t		rpmbs; /* Really a bitfield */

	/** Extended Device Self-test Time */
	uint16_t		edstt;

	/** Device Self-test Options */
	uint8_t			dsto; /* Really a bitfield */

	/** Firmware Update Granularity */
	uint8_t			fwug;

	/** Keep Alive Support */
	uint16_t		kas;

	/** Host Controlled Thermal Management Attributes */
	uint16_t		hctma; /* Really a bitfield */

	/** Minimum Thermal Management Temperature */
	uint16_t		mntmt;

	/** Maximum Thermal Management Temperature */
	uint16_t		mxtmt;

	/** Sanitize Capabilities */
	uint32_t		sanicap; /* Really a bitfield */

	uint8_t reserved3[180];
	/* bytes 512-703: nvm command set attributes */

	/** submission queue entry size */
	struct {
		uint8_t		min : 4;
		uint8_t		max : 4;
	} __packed sqes;

	/** completion queue entry size */
	struct {
		uint8_t		min : 4;
		uint8_t		max : 4;
	} __packed cqes;

	/** Maximum Outstanding Commands */
	uint16_t		maxcmd;

	/** number of namespaces */
	uint32_t		nn;

	/** optional nvm command support */
	struct {
		uint16_t	compare : 1;
		uint16_t	write_unc : 1;
		uint16_t	dsm: 1;
		uint16_t	reserved: 13;
	} __packed oncs;

	/** fused operation support */
	uint16_t		fuses;

	/** format nvm attributes */
	uint8_t			fna;

	/** volatile write cache */
	struct {
		uint8_t		present : 1;
		uint8_t		reserved : 7;
	} __packed vwc;

	/* TODO: flesh out remaining nvm command set attributes */
	uint8_t			reserved5[178];

	/* bytes 704-2047: i/o command set attributes */
	uint8_t			reserved6[1344];

	/* bytes 2048-3071: power state descriptors */
	struct nvme_power_state power_state[32];

	/* bytes 3072-4095: vendor specific */
	uint8_t			vs[1024];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_controller_data) == 4096, "bad size for nvme_controller_data");

struct nvme_namespace_data {

	/** namespace size */
	uint64_t		nsze;

	/** namespace capacity */
	uint64_t		ncap;

	/** namespace utilization */
	uint64_t		nuse;

	/** namespace features */
	struct {
		/** thin provisioning */
		uint8_t		thin_prov : 1;
		uint8_t		reserved1 : 7;
	} __packed nsfeat;

	/** number of lba formats */
	uint8_t			nlbaf;

	uint8_t			flbas;

	/** metadata capabilities */
	struct {
		/* metadata can be transferred as part of data prp list */
		uint8_t		extended  : 1;

		/* metadata can be transferred with separate metadata pointer */
		uint8_t		pointer   : 1;

		uint8_t		reserved3 : 6;
	} __packed mc;

	/** end-to-end data protection capabilities */
	struct {
		/* protection information type 1 */
		uint8_t		pit1     : 1;

		/* protection information type 2 */
		uint8_t		pit2     : 1;

		/* protection information type 3 */
		uint8_t		pit3     : 1;

		/* first eight bytes of metadata */
		uint8_t		md_start : 1;

		/* last eight bytes of metadata */
		uint8_t		md_end   : 1;
	} __packed dpc;

	/** end-to-end data protection type settings */
	struct {
		/* protection information type */
		uint8_t		pit       : 3;

		/* 1 == protection info transferred at start of metadata */
		/* 0 == protection info transferred at end of metadata */
		uint8_t		md_start  : 1;

		uint8_t		reserved4 : 4;
	} __packed dps;

	uint8_t			reserved5[98];

	uint32_t		lbaf[16];

	uint8_t			reserved6[192];

	uint8_t			vendor_specific[3712];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_namespace_data) == 4096, "bad size for nvme_namepsace_data");

enum nvme_log_page {

	/* 0x00 - reserved */
	NVME_LOG_ERROR			= 0x01,
	NVME_LOG_HEALTH_INFORMATION	= 0x02,
	NVME_LOG_FIRMWARE_SLOT		= 0x03,
	NVME_LOG_CHANGED_NAMESPACE	= 0x04,
	NVME_LOG_COMMAND_EFFECT		= 0x05,
	/* 0x06-0x7F - reserved */
	/* 0x80-0xBF - I/O command set specific */
	NVME_LOG_RES_NOTIFICATION	= 0x80,
	/* 0xC0-0xFF - vendor specific */

	/*
	 * The following are Intel Specific log pages, but they seem
	 * to be widely implemented.
	 */
	INTEL_LOG_READ_LAT_LOG		= 0xc1,
	INTEL_LOG_WRITE_LAT_LOG		= 0xc2,
	INTEL_LOG_TEMP_STATS		= 0xc5,
	INTEL_LOG_ADD_SMART		= 0xca,
	INTEL_LOG_DRIVE_MKT_NAME	= 0xdd,

	/*
	 * HGST log page, with lots ofs sub pages.
	 */
	HGST_INFO_LOG			= 0xc1,
};

struct nvme_error_information_entry {

	uint64_t		error_count;
	uint16_t		sqid;
	uint16_t		cid;
	uint16_t		status;
	uint16_t		error_location;
	uint64_t		lba;
	uint32_t		nsid;
	uint8_t			vendor_specific;
	uint8_t			reserved[35];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_error_information_entry) == 64, "bad size for nvme_error_information_entry");

union nvme_critical_warning_state {

	uint8_t		raw;

	struct {
		uint8_t	available_spare		: 1;
		uint8_t	temperature		: 1;
		uint8_t	device_reliability	: 1;
		uint8_t	read_only		: 1;
		uint8_t	volatile_memory_backup	: 1;
		uint8_t	reserved		: 3;
	} __packed bits;
} __packed;

_Static_assert(sizeof(union nvme_critical_warning_state) == 1, "bad size for nvme_critical_warning_state");

struct nvme_health_information_page {

	union nvme_critical_warning_state	critical_warning;

	uint16_t		temperature;
	uint8_t			available_spare;
	uint8_t			available_spare_threshold;
	uint8_t			percentage_used;

	uint8_t			reserved[26];

	/*
	 * Note that the following are 128-bit values, but are
	 *  defined as an array of 2 64-bit values.
	 */
	/* Data Units Read is always in 512-byte units. */
	uint64_t		data_units_read[2];
	/* Data Units Written is always in 512-byte units. */
	uint64_t		data_units_written[2];
	/* For NVM command set, this includes Compare commands. */
	uint64_t		host_read_commands[2];
	uint64_t		host_write_commands[2];
	/* Controller Busy Time is reported in minutes. */
	uint64_t		controller_busy_time[2];
	uint64_t		power_cycles[2];
	uint64_t		power_on_hours[2];
	uint64_t		unsafe_shutdowns[2];
	uint64_t		media_errors[2];
	uint64_t		num_error_info_log_entries[2];
	uint32_t		warning_temp_time;
	uint32_t		error_temp_time;
	uint16_t		temp_sensor[8];

	uint8_t			reserved2[296];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_health_information_page) == 512, "bad size for nvme_health_information_page");

struct nvme_firmware_page {

	struct {
		uint8_t	slot		: 3; /* slot for current FW */
		uint8_t	reserved	: 5;
	} __packed afi;

	uint8_t			reserved[7];
	uint64_t		revision[7]; /* revisions for 7 slots */
	uint8_t			reserved2[448];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_firmware_page) == 512, "bad size for nvme_firmware_page");

struct intel_log_temp_stats
{
	uint64_t	current;
	uint64_t	overtemp_flag_last;
	uint64_t	overtemp_flag_life;
	uint64_t	max_temp;
	uint64_t	min_temp;
	uint64_t	_rsvd[5];
	uint64_t	max_oper_temp;
	uint64_t	min_oper_temp;
	uint64_t	est_offset;
} __packed __aligned(4);

_Static_assert(sizeof(struct intel_log_temp_stats) == 13 * 8, "bad size for intel_log_temp_stats");

#define NVME_TEST_MAX_THREADS	128

struct nvme_io_test {

	enum nvme_nvm_opcode	opc;
	uint32_t		size;
	uint32_t		time;	/* in seconds */
	uint32_t		num_threads;
	uint32_t		flags;
	uint64_t		io_completed[NVME_TEST_MAX_THREADS];
};

enum nvme_io_test_flags {

	/*
	 * Specifies whether dev_refthread/dev_relthread should be
	 *  called during NVME_BIO_TEST.  Ignored for other test
	 *  types.
	 */
	NVME_TEST_FLAG_REFTHREAD =	0x1,
};

struct nvme_pt_command {

	/*
	 * cmd is used to specify a passthrough command to a controller or
	 *  namespace.
	 *
	 * The following fields from cmd may be specified by the caller:
	 *	* opc  (opcode)
	 *	* nsid (namespace id) - for admin commands only
	 *	* cdw10-cdw15
	 *
	 * Remaining fields must be set to 0 by the caller.
	 */
	struct nvme_command	cmd;

	/*
	 * cpl returns completion status for the passthrough command
	 *  specified by cmd.
	 *
	 * The following fields will be filled out by the driver, for
	 *  consumption by the caller:
	 *	* cdw0
	 *	* status (except for phase)
	 *
	 * Remaining fields will be set to 0 by the driver.
	 */
	struct nvme_completion	cpl;

	/* buf is the data buffer associated with this passthrough command. */
	void *			buf;

	/*
	 * len is the length of the data buffer associated with this
	 *  passthrough command.
	 */
	uint32_t		len;

	/*
	 * is_read = 1 if the passthrough command will read data into the
	 *  supplied buffer from the controller.
	 *
	 * is_read = 0 if the passthrough command will write data from the
	 *  supplied buffer to the controller.
	 */
	uint32_t		is_read;

	/*
	 * driver_lock is used by the driver only.  It must be set to 0
	 *  by the caller.
	 */
	struct mtx *		driver_lock;
};

#define nvme_completion_is_error(cpl)					\
	(NVME_STATUS_GET_SC(le16toh((cpl)->status)) != 0 || NVME_STATUS_GET_SCT(le16toh((cpl)->status)) != 0)

void	nvme_strvis(uint8_t *dst, const uint8_t *src, int dstlen, int srclen);

#ifdef _KERNEL

struct bio;

struct nvme_namespace;
struct nvme_controller;
struct nvme_consumer;

typedef void (*nvme_cb_fn_t)(void *, const struct nvme_completion *);

typedef void *(*nvme_cons_ns_fn_t)(struct nvme_namespace *, void *);
typedef void *(*nvme_cons_ctrlr_fn_t)(struct nvme_controller *);
typedef void (*nvme_cons_async_fn_t)(void *, const struct nvme_completion *,
				     uint32_t, void *, uint32_t);
typedef void (*nvme_cons_fail_fn_t)(void *);

enum nvme_namespace_flags {
	NVME_NS_DEALLOCATE_SUPPORTED	= 0x1,
	NVME_NS_FLUSH_SUPPORTED		= 0x2,
};

int	nvme_ctrlr_passthrough_cmd(struct nvme_controller *ctrlr,
				   struct nvme_pt_command *pt,
				   uint32_t nsid, int is_user_buffer,
				   int is_admin_cmd);

/* Admin functions */
void	nvme_ctrlr_cmd_set_feature(struct nvme_controller *ctrlr,
				   uint8_t feature, uint32_t cdw11,
				   void *payload, uint32_t payload_size,
				   nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_get_feature(struct nvme_controller *ctrlr,
				   uint8_t feature, uint32_t cdw11,
				   void *payload, uint32_t payload_size,
				   nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_get_log_page(struct nvme_controller *ctrlr,
				    uint8_t log_page, uint32_t nsid,
				    void *payload, uint32_t payload_size,
				    nvme_cb_fn_t cb_fn, void *cb_arg);

/* NVM I/O functions */
int	nvme_ns_cmd_write(struct nvme_namespace *ns, void *payload,
			  uint64_t lba, uint32_t lba_count, nvme_cb_fn_t cb_fn,
			  void *cb_arg);
int	nvme_ns_cmd_write_bio(struct nvme_namespace *ns, struct bio *bp,
			      nvme_cb_fn_t cb_fn, void *cb_arg);
int	nvme_ns_cmd_read(struct nvme_namespace *ns, void *payload,
			 uint64_t lba, uint32_t lba_count, nvme_cb_fn_t cb_fn,
			 void *cb_arg);
int	nvme_ns_cmd_read_bio(struct nvme_namespace *ns, struct bio *bp,
			      nvme_cb_fn_t cb_fn, void *cb_arg);
int	nvme_ns_cmd_deallocate(struct nvme_namespace *ns, void *payload,
			       uint8_t num_ranges, nvme_cb_fn_t cb_fn,
			       void *cb_arg);
int	nvme_ns_cmd_flush(struct nvme_namespace *ns, nvme_cb_fn_t cb_fn,
			  void *cb_arg);
int	nvme_ns_dump(struct nvme_namespace *ns, void *virt, off_t offset,
		     size_t len);

/* Registration functions */
struct nvme_consumer *	nvme_register_consumer(nvme_cons_ns_fn_t    ns_fn,
					       nvme_cons_ctrlr_fn_t ctrlr_fn,
					       nvme_cons_async_fn_t async_fn,
					       nvme_cons_fail_fn_t  fail_fn);
void		nvme_unregister_consumer(struct nvme_consumer *consumer);

/* Controller helper functions */
device_t	nvme_ctrlr_get_device(struct nvme_controller *ctrlr);
const struct nvme_controller_data *
		nvme_ctrlr_get_data(struct nvme_controller *ctrlr);

/* Namespace helper functions */
uint32_t	nvme_ns_get_max_io_xfer_size(struct nvme_namespace *ns);
uint32_t	nvme_ns_get_sector_size(struct nvme_namespace *ns);
uint64_t	nvme_ns_get_num_sectors(struct nvme_namespace *ns);
uint64_t	nvme_ns_get_size(struct nvme_namespace *ns);
uint32_t	nvme_ns_get_flags(struct nvme_namespace *ns);
const char *	nvme_ns_get_serial_number(struct nvme_namespace *ns);
const char *	nvme_ns_get_model_number(struct nvme_namespace *ns);
const struct nvme_namespace_data *
		nvme_ns_get_data(struct nvme_namespace *ns);
uint32_t	nvme_ns_get_stripesize(struct nvme_namespace *ns);

int	nvme_ns_bio_process(struct nvme_namespace *ns, struct bio *bp,
			    nvme_cb_fn_t cb_fn);

/*
 * Command building helper functions -- shared with CAM
 * These functions assume allocator zeros out cmd structure
 * CAM's xpt_get_ccb and the request allocator for nvme both
 * do zero'd allocations.
 */
static inline
void	nvme_ns_flush_cmd(struct nvme_command *cmd, uint32_t nsid)
{

	cmd->opc_fuse = htole16(NVME_CMD_SET_OPC(NVME_OPC_FLUSH));
	cmd->nsid = htole32(nsid);
}

static inline
void	nvme_ns_rw_cmd(struct nvme_command *cmd, uint32_t rwcmd, uint32_t nsid,
    uint64_t lba, uint32_t count)
{
	cmd->opc_fuse = htole16(NVME_CMD_SET_OPC(rwcmd));
	cmd->nsid = htole32(nsid);
	cmd->cdw10 = htole32(lba & 0xffffffffu);
	cmd->cdw11 = htole32(lba >> 32);
	cmd->cdw12 = htole32(count-1);
}

static inline
void	nvme_ns_write_cmd(struct nvme_command *cmd, uint32_t nsid,
    uint64_t lba, uint32_t count)
{
	nvme_ns_rw_cmd(cmd, NVME_OPC_WRITE, nsid, lba, count);
}

static inline
void	nvme_ns_read_cmd(struct nvme_command *cmd, uint32_t nsid,
    uint64_t lba, uint32_t count)
{
	nvme_ns_rw_cmd(cmd, NVME_OPC_READ, nsid, lba, count);
}

static inline
void	nvme_ns_trim_cmd(struct nvme_command *cmd, uint32_t nsid,
    uint32_t num_ranges)
{
	cmd->opc_fuse = htole16(NVME_CMD_SET_OPC(NVME_OPC_DATASET_MANAGEMENT));
	cmd->nsid = htole32(nsid);
	cmd->cdw10 = htole32(num_ranges - 1);
	cmd->cdw11 = htole32(NVME_DSM_ATTR_DEALLOCATE);
}

extern int nvme_use_nvd;

#endif /* _KERNEL */

#endif /* __NVME_H__ */
