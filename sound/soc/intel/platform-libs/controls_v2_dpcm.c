/*
 *  controls_v2_dpcm.c - Intel MID Platform driver DPCM ALSA controls for Mrfld
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "../platform_ipc_v2.h"
#include "../sst_platform.h"
#include "../sst_platform_pvt.h"
#include "controls_v2.h"
#include "sst_widgets.h"

static inline void sst_fill_byte_control(char *param,
					 u8 ipc_msg, u8 block,
					 u8 task_id, u8 pipe_id,
					 u16 len, void *cmd_data)
{

	struct snd_sst_bytes_v2 *byte_data = (struct snd_sst_bytes_v2 *)param;
	byte_data->type = SST_CMD_BYTES_SET;
	byte_data->ipc_msg = ipc_msg;
	byte_data->block = block;
	byte_data->task_id = task_id;
	byte_data->pipe_id = pipe_id;

	if (len > SST_MAX_BIN_BYTES - sizeof(*byte_data)) {
		pr_err("%s: command length too big (%u)", __func__, len);
		len = SST_MAX_BIN_BYTES - sizeof(*byte_data);
		WARN_ON(1); /* this happens only if code is wrong */
	}
	byte_data->len = len;
	memcpy(byte_data->bytes, cmd_data, len);
	print_hex_dump_bytes("writing to lpe: ", DUMP_PREFIX_OFFSET,
			     byte_data, len + sizeof(*byte_data));
}

static int sst_fill_and_send_cmd(struct sst_data *sst,
				 u8 ipc_msg, u8 block, u8 task_id, u8 pipe_id,
				 void *cmd_data, u16 len)
{
	int ret = 0;

	mutex_lock(&sst->lock);
	sst_fill_byte_control(sst->byte_stream, ipc_msg, block, task_id, pipe_id,
			      len, cmd_data);
	ret = sst_dsp->ops->set_generic_params(SST_SET_BYTE_STREAM,
					       sst->byte_stream);
	mutex_unlock(&sst->lock);

	return ret;
}

static int sst_probe_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct sst_probe_value *v = (void *)kcontrol->private_value;

	ucontrol->value.enumerated.item[0] = v->val;
	return 0;
}

static int sst_probe_put(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct sst_probe_value *v = (void *)kcontrol->private_value;
	const struct soc_enum *e = v->p_enum;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;
	v->val = ucontrol->value.enumerated.item[0];
	return 0;
}

int sst_probe_enum_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct sst_probe_value *v = (void *)kcontrol->private_value;
	const struct soc_enum *e = v->p_enum;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = e->max;

	if (uinfo->value.enumerated.item > e->max - 1)
		uinfo->value.enumerated.item = e->max - 1;
	strcpy(uinfo->value.enumerated.name,
		e->texts[uinfo->value.enumerated.item]);
	return 0;
}

/*
 * slot map value is a bitfield where each bit represents a FW channel
 *
 *			3 2 1 0		# 0 = codec0, 1 = codec1
 *			RLRLRLRL	# 3, 4 = reserved
 *
 * e.g. slot 0 rx map =	00001100b -> data from slot 0 goes into codec_in1 L,R
 */
static u8 sst_ssp_slot_map[SST_MAX_TDM_SLOTS] = {
	0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, /* default rx map */
};

/*
 * channel map value is a bitfield where each bit represents a slot
 *
 *			  76543210	# 0 = slot 0, 1 = slot 1
 *
 * e.g. codec1_0 tx map = 00000101b -> data from codec_out1_0 goes into slot 0, 2
 */
static u8 sst_ssp_channel_map[SST_MAX_TDM_SLOTS] = {
	0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, /* default tx map */
};

static int sst_slot_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (void *)kcontrol->private_value;
	unsigned int ctl_no = e->reg;
	unsigned int is_tx = e->reg2;
	unsigned int val, mux;
	u8 *map = is_tx ? sst_ssp_channel_map : sst_ssp_slot_map;

	val = 1 << ctl_no;
	/* search which slot/channel has this bit set - there should be only one */
	for (mux = e->max; mux > 0;  mux--)
		if (map[mux - 1] & val)
			break;

	ucontrol->value.enumerated.item[0] = mux;
	pr_debug("%s: %s - %s map = %#x\n", __func__, is_tx ? "tx channel" : "rx slot",
		 e->texts[mux], mux ? map[mux - 1] : -1);
	return 0;
}

/*
 * (de)interleaver controls are defined in opposite sense to be user-friendly
 *
 * Instead of the enum value being the value set to the register, it is the
 * register address; and the kcontrol_no is the value written to the register.
 *
 * This means that whenever an enum is set, we need to clear the bit
 * for that kcontrol_no for all the interleaver OR deinterleaver registers
 */
static int sst_slot_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (void *)kcontrol->private_value;
	int i;
	unsigned int ctl_no = e->reg;
	unsigned int is_tx = e->reg2;
	unsigned int slot_channel_no;
	unsigned int val, mux;
	u8 *map = is_tx ? sst_ssp_channel_map : sst_ssp_slot_map;

	val = 1 << ctl_no;
	mux = ucontrol->value.enumerated.item[0];
	if (mux > e->max - 1)
		return -EINVAL;

	/* first clear all registers of this bit */
	for (i = 0; i < e->max; i++)
		map[i] &= ~val;

	if (mux == 0) /* kctl set to 'none' */
		return 0;

	/* offset by one to take "None" into account */
	slot_channel_no = mux - 1;
	map[slot_channel_no] |= val;

	pr_debug("%s: %s %s map = %#x\n", __func__, is_tx ? "tx channel" : "rx slot",
		 e->texts[mux], map[slot_channel_no]);
	return 0;
}

/* assumes a boolean mux */
static inline bool get_mux_state(struct sst_data *sst, unsigned int reg, unsigned int shift)
{
	return (sst_reg_read(sst, reg, shift, 1) == 1);
}

static int sst_mux_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct sst_data *sst = snd_soc_platform_get_drvdata(widget->platform);
	struct soc_enum *e = (void *)kcontrol->private_value;
	unsigned int max = e->max - 1;

	ucontrol->value.enumerated.item[0] = sst_reg_read(sst, e->reg, e->shift_l, max);
	return 0;
}

static int sst_mux_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct sst_data *sst = snd_soc_platform_get_drvdata(widget->platform);
	struct soc_enum *e = (void *)kcontrol->private_value;
	struct snd_soc_dapm_update update;
	unsigned int max = e->max - 1;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int mux, val;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	mux = ucontrol->value.enumerated.item[0];
	val = sst_reg_write(sst, e->reg, e->shift_l, max, mux);

	pr_debug("%s: reg[%d] = %#x\n", __func__, e->reg, val);

	widget->value = val;
	update.kcontrol = kcontrol;
	update.widget = widget;
	update.reg = e->reg;
	update.mask = mask;
	update.val = val;

	widget->dapm->update = &update;
	snd_soc_dapm_mux_update_power(widget, kcontrol, mux, e);
	widget->dapm->update = NULL;
	return 0;
}

static int sst_mode_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	struct sst_data *sst = snd_soc_platform_get_drvdata(platform);
	struct soc_enum *e = (void *)kcontrol->private_value;
	unsigned int max = e->max - 1;

	ucontrol->value.enumerated.item[0] = sst_reg_read(sst, e->reg, e->shift_l, max);
	return 0;
}

static int sst_mode_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	struct sst_data *sst = snd_soc_platform_get_drvdata(platform);
	struct soc_enum *e = (void *)kcontrol->private_value;
	unsigned int max = e->max - 1;
	unsigned int val;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	val = sst_reg_write(sst, e->reg, e->shift_l, max, ucontrol->value.enumerated.item[0]);
	pr_debug("%s: reg[%d] - %#x\n", __func__, e->reg, val);
	return 0;
}

static void sst_send_algo_cmd(struct sst_data *sst,
			      struct sst_algo_control *bc)
{
	int len;
	struct sst_cmd_set_params *cmd;

	len = sizeof(cmd->dst) + sizeof(cmd->command_id) + bc->max;

	cmd = kzalloc(len + bc->max, GFP_KERNEL);
	if (cmd == NULL) {
		pr_err("Failed to send cmd, kzalloc failed\n");
		return;
	}

	SST_FILL_DESTINATION(2, cmd->dst, bc->pipe_id, bc->module_id);
	cmd->command_id = bc->cmd_id;
	memcpy(cmd->params, bc->params, bc->max);

	sst_fill_and_send_cmd(sst, SST_IPC_IA_SET_PARAMS, SST_FLAG_BLOCKED,
				bc->task_id, 0, cmd, len);
	kfree(cmd);

}

static void sst_find_and_send_pipe_algo(struct snd_soc_platform *platform,
					struct snd_soc_dapm_widget *w)
{
	struct sst_algo_control *bc;
	struct sst_data *sst = snd_soc_platform_get_drvdata(platform);
	struct sst_ids *ids = w->priv;
	struct module *algo = NULL;

	pr_debug("Enter:%s, widget=%s\n", __func__, w->name);

	list_for_each_entry(algo, &ids->algo_list, node) {
			bc = (void *)algo->kctl->private_value;

			pr_debug("Found algo control name =%s pipe=%s\n", algo->kctl->id.name,  w->name);
			sst_send_algo_cmd(sst, bc);
	}
}

int sst_algo_bytes_ctl_info(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	struct sst_algo_control *bc = (void *)kcontrol->private_value;
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = bc->max;

	if (bc->params == NULL) {
		bc->params = devm_kzalloc(platform->dev, bc->max, GFP_KERNEL);
		if (bc->params == NULL) {
			pr_err("kzalloc failed\n");
			return -ENOMEM;
		}
	}
	return 0;
}

static int sst_algo_control_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct sst_algo_control *bc = (void *)kcontrol->private_value;

	pr_debug("in %s\n", __func__);
	switch (bc->type) {
	case SST_ALGO_PARAMS:
		if (bc->params)
			memcpy(ucontrol->value.bytes.data, bc->params, bc->max);
		break;
	case SST_ALGO_BYPASS:
		ucontrol->value.integer.value[0] = bc->bypass ? 1 : 0;
		pr_debug("%s: bypass  %d\n", __func__, bc->bypass);
		break;
	default:
		pr_err("Invalid Input- algo type:%d\n", bc->type);
		return -EINVAL;

	}
	return 0;
}

static int sst_algo_control_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	struct sst_data *sst = snd_soc_platform_get_drvdata(platform);
	struct sst_algo_control *bc = (void *)kcontrol->private_value;

	pr_debug("in %s control_name=%s\n", __func__, kcontrol->id.name);
	switch (bc->type) {
	case SST_ALGO_PARAMS:
		if (bc->params)
			memcpy(bc->params, ucontrol->value.bytes.data, bc->max);
		break;
	case SST_ALGO_BYPASS:
		bc->bypass = !!ucontrol->value.integer.value[0];
		pr_debug("%s: Mute %d\n", __func__, bc->bypass);
		break;
	default:
		pr_err("Invalid Input- algo type:%ld\n", ucontrol->value.integer.value[0]);
		return -EINVAL;
	}
	/*if pipe is enabled, need to send the algo params from here */
	if (bc->w && bc->w->power)
		sst_send_algo_cmd(sst, bc);

	return 0;
}

static int sst_gain_ctl_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct sst_gain_mixer_control *mc = (void *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = mc->stereo ? 2 : 1;
	uinfo->value.integer.min = mc->min;
	uinfo->value.integer.max = mc->max;
	return 0;
}

static void sst_send_gain_cmd(struct sst_data *sst, struct sst_gain_value *gv,
			      u16 task_id, u16 loc_id, u16 module_id, int mute)
{
	struct sst_cmd_set_gain_dual cmd;
	pr_debug("%s", __func__);

	cmd.header.command_id = MMX_SET_GAIN;
	SST_FILL_DEFAULT_DESTINATION(cmd.header.dst);
	cmd.gain_cell_num = 1;

	if (mute || gv->mute) {
		cmd.cell_gains[0].cell_gain_left = SST_GAIN_MIN_VALUE;
		cmd.cell_gains[0].cell_gain_right = SST_GAIN_MIN_VALUE;
	} else {
		cmd.cell_gains[0].cell_gain_left = gv->l_gain;
		cmd.cell_gains[0].cell_gain_right = gv->r_gain;
	}
	SST_FILL_DESTINATION(2, cmd.cell_gains[0].dest,
			     loc_id, module_id);
	cmd.cell_gains[0].gain_time_constant = gv->ramp_duration;

	cmd.header.length = sizeof(struct sst_cmd_set_gain_dual)
				- sizeof(struct sst_dsp_header);

	sst_fill_and_send_cmd(sst, SST_IPC_IA_SET_PARAMS, SST_FLAG_BLOCKED,
			      task_id, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);
}

static int sst_gain_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct sst_gain_mixer_control *mc = (void *)kcontrol->private_value;
	struct sst_gain_value *gv = mc->gain_val;

	switch (mc->type) {
	case SST_GAIN_TLV:
		ucontrol->value.integer.value[0] = gv->l_gain;
		ucontrol->value.integer.value[1] = gv->r_gain;
		pr_debug("%s: Volume %d, %d\n", __func__, gv->l_gain, gv->r_gain);
		break;
	case SST_GAIN_MUTE:
		ucontrol->value.integer.value[0] = gv->mute ? 1 : 0;
		pr_debug("%s: Mute %d\n", __func__, gv->mute);
		break;
	case SST_GAIN_RAMP_DURATION:
		ucontrol->value.integer.value[0] = gv->ramp_duration;
		pr_debug("%s: RampDuration %d\n", __func__, gv->ramp_duration);
		break;
	default:
		pr_err("Invalid Input- gain type:%d\n", mc->type);
		return -EINVAL;
	};
	return 0;
}

static int sst_gain_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	struct sst_data *sst = snd_soc_platform_get_drvdata(platform);
	struct sst_gain_mixer_control *mc = (void *)kcontrol->private_value;
	struct sst_gain_value *gv = mc->gain_val;

	switch (mc->type) {
	case SST_GAIN_TLV:
		gv->l_gain = ucontrol->value.integer.value[0];
		gv->r_gain = ucontrol->value.integer.value[1];
		pr_debug("%s: Volume %d, %d\n", __func__, gv->l_gain, gv->r_gain);
		break;
	case SST_GAIN_MUTE:
		gv->mute = !!ucontrol->value.integer.value[0];
		pr_debug("%s: Mute %d\n", __func__, gv->mute);
		break;
	case SST_GAIN_RAMP_DURATION:
		gv->ramp_duration = ucontrol->value.integer.value[0];
		pr_debug("%s: RampDuration %d\n", __func__, gv->ramp_duration);
		break;
	default:
		pr_err("Invalid Input- gain type:%d\n", mc->type);
		return -EINVAL;
	};

	if (mc->w && mc->w->power)
		sst_send_gain_cmd(sst, gv, mc->task_id,
				mc->pipe_id | mc->instance_id, mc->module_id, 0);
	return 0;
}

static const DECLARE_TLV_DB_SCALE(sst_gain_tlv_common, SST_GAIN_MIN_VALUE * 10, 10, 0);

/* Look up table to convert MIXER SW bit regs to SWM inputs */
static const uint swm_mixer_input_ids[SST_SWM_INPUT_COUNT] = {
	[SST_IP_MODEM]		= SST_SWM_IN_MODEM,
	[SST_IP_BT]		= SST_SWM_IN_BT,
	[SST_IP_CODEC0]		= SST_SWM_IN_CODEC0,
	[SST_IP_CODEC1]		= SST_SWM_IN_CODEC1,
	[SST_IP_LOOP0]		= SST_SWM_IN_SPROT_LOOP,
	[SST_IP_LOOP1]		= SST_SWM_IN_MEDIA_LOOP1,
	[SST_IP_LOOP2]		= SST_SWM_IN_MEDIA_LOOP2,
	[SST_IP_SIDETONE]	= SST_SWM_IN_SIDETONE,
	[SST_IP_TXSPEECH]	= SST_SWM_IN_TXSPEECH,
	[SST_IP_SPEECH]		= SST_SWM_IN_SPEECH,
	[SST_IP_TONE]		= SST_SWM_IN_TONE,
	[SST_IP_VOIP]		= SST_SWM_IN_VOIP,
	[SST_IP_PCM0]		= SST_SWM_IN_PCM0,
	[SST_IP_PCM1]		= SST_SWM_IN_PCM1,
	[SST_IP_LOW_PCM0]	= SST_SWM_IN_LOW_PCM0,
	[SST_IP_FM]		= SST_SWM_IN_FM,
	[SST_IP_MEDIA0]		= SST_SWM_IN_MEDIA0,
	[SST_IP_MEDIA1]		= SST_SWM_IN_MEDIA1,
	[SST_IP_MEDIA2]		= SST_SWM_IN_MEDIA2,
	[SST_IP_MEDIA3]		= SST_SWM_IN_MEDIA3,
};

static int fill_swm_input(struct swm_input_ids *swm_input, unsigned int reg)
{
	uint i, is_set, nb_inputs = 0;
	u16 input_loc_id;

	pr_debug("%s:reg value:%#x\n", __func__, reg);
	for (i = 0; i < SST_SWM_INPUT_COUNT; i++) {
		is_set = reg & BIT(i);
		if (!is_set)
			continue;

		input_loc_id = swm_mixer_input_ids[i];
		SST_FILL_DESTINATION(2, swm_input->input_id,
				     input_loc_id, SST_DEFAULT_MODULE_ID);
		nb_inputs++;
		swm_input++;
		pr_debug("input id:%#x, nb_inputs:%d\n", input_loc_id, nb_inputs);

		if (nb_inputs == SST_CMD_SWM_MAX_INPUTS) {
			pr_warn("%s: SET_SWM cmd max inputs reached", __func__);
			break;
		}
	}
	return nb_inputs;
}

static void sst_set_pipe_gain(struct sst_ids *ids, struct sst_data *sst, int mute)
{
	struct sst_gain_mixer_control *mc;
	struct sst_gain_value *gv;
	struct module *gain = NULL;

	list_for_each_entry(gain, &ids->gain_list, node) {
		struct snd_kcontrol *kctl = gain->kctl;

		pr_debug("control name=%s", kctl->id.name);
		mc = (void *)kctl->private_value;
		gv = mc->gain_val;

		sst_send_gain_cmd(sst, gv, mc->task_id,
				mc->pipe_id | mc->instance_id, mc->module_id, mute);
	}
}

static int sst_swm_mixer_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	struct sst_cmd_set_swm cmd;
	struct sst_data *sst = snd_soc_platform_get_drvdata(w->platform);
	struct sst_ids *ids = w->priv;
	bool set_mixer = false;
	int val = sst->widget[ids->reg];

	pr_debug("%s: widget=%s\n", __func__, w->name);
	pr_debug("%s: reg[%d] = %#x\n", __func__, ids->reg, val);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
	case SND_SOC_DAPM_POST_PMD:
		set_mixer = true;
		break;
	case SND_SOC_DAPM_POST_REG:
		if (w->power)
			set_mixer = true;
		break;
	default:
		set_mixer = false;
	}

	if (set_mixer == false)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event) ||
	    event == SND_SOC_DAPM_POST_REG)
		cmd.switch_state = SST_SWM_ON;
	else
		cmd.switch_state = SST_SWM_OFF;

	SST_FILL_DEFAULT_DESTINATION(cmd.header.dst);
	/* MMX_SET_SWM == SBA_SET_SWM */
	cmd.header.command_id = SBA_SET_SWM;

	SST_FILL_DESTINATION(2, cmd.output_id,
			     ids->location_id, SST_DEFAULT_MODULE_ID);
	cmd.nb_inputs =	fill_swm_input(&cmd.input[0], val);
	cmd.header.length = offsetof(struct sst_cmd_set_swm, input) - sizeof(struct sst_dsp_header)
				+ (cmd.nb_inputs * sizeof(cmd.input[0]));

	sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
			      ids->task_id, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);
	return 0;
}

/* SBA mixers - 16 inputs */
#define SST_SBA_DECLARE_MIX_CONTROLS(kctl_name, mixer_reg)			\
	static const struct snd_kcontrol_new kctl_name[] = {			\
		SOC_SINGLE_EXT("modem_in", mixer_reg, SST_IP_MODEM, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("bt_in", mixer_reg, SST_IP_BT, 1, 0,		\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("codec_in0", mixer_reg, SST_IP_CODEC0, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("codec_in1", mixer_reg, SST_IP_CODEC1, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("sprot_loop_in", mixer_reg, SST_IP_LOOP0, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("media_loop1_in", mixer_reg, SST_IP_LOOP1, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("media_loop2_in", mixer_reg, SST_IP_LOOP2, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("sidetone_in", mixer_reg, SST_IP_SIDETONE, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("txspeech_in", mixer_reg, SST_IP_TXSPEECH, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("speech_in", mixer_reg, SST_IP_SPEECH, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("tone_in", mixer_reg, SST_IP_TONE, 1, 0,		\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("voip_in", mixer_reg, SST_IP_VOIP, 1, 0,		\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("pcm0_in", mixer_reg, SST_IP_PCM0, 1, 0,		\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("pcm1_in", mixer_reg, SST_IP_PCM1, 1, 0,		\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("low_pcm0_in", mixer_reg, SST_IP_LOW_PCM0, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("fm_in", mixer_reg, SST_IP_FM, 1, 0,		\
				sst_mix_get, sst_mix_put),			\
	}

#define SST_SBA_MIXER_GRAPH_MAP(mix_name)			\
	{ mix_name, "modem_in",		"modem_in" },		\
	{ mix_name, "bt_in",		"bt_in" },		\
	{ mix_name, "codec_in0",	"codec_in0" },		\
	{ mix_name, "codec_in1",	"codec_in1" },		\
	{ mix_name, "sprot_loop_in",	"sprot_loop_in" },	\
	{ mix_name, "media_loop1_in",	"media_loop1_in" },	\
	{ mix_name, "media_loop2_in",	"media_loop2_in" },	\
	{ mix_name, "sidetone_in",	"sidetone_in" },	\
	{ mix_name, "txspeech_in",	"txspeech_in" },	\
	{ mix_name, "speech_in",	"speech_in" },		\
	{ mix_name, "tone_in",		"tone_in" },		\
	{ mix_name, "voip_in",		"voip_in" },		\
	{ mix_name, "pcm0_in",		"pcm0_in" },		\
	{ mix_name, "pcm1_in",		"pcm1_in" },		\
	{ mix_name, "low_pcm0_in",	"low_pcm0_in" },	\
	{ mix_name, "fm_in",		"fm_in" }

#define SST_MMX_DECLARE_MIX_CONTROLS(kctl_name, mixer_reg)			\
	static const struct snd_kcontrol_new kctl_name[] = {			\
		SOC_SINGLE_EXT("media0_in", mixer_reg, SST_IP_MEDIA0, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("media1_in", mixer_reg, SST_IP_MEDIA1, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("media2_in", mixer_reg, SST_IP_MEDIA2, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
		SOC_SINGLE_EXT("media3_in", mixer_reg, SST_IP_MEDIA3, 1, 0,	\
				sst_mix_get, sst_mix_put),			\
	}

SST_MMX_DECLARE_MIX_CONTROLS(sst_mix_media0_controls, SST_MIX_MEDIA0);
SST_MMX_DECLARE_MIX_CONTROLS(sst_mix_media1_controls, SST_MIX_MEDIA1);

/* 18 SBA mixers */
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_pcm0_controls, SST_MIX_PCM0);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_pcm1_controls, SST_MIX_PCM1);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_pcm2_controls, SST_MIX_PCM2);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_sprot_l0_controls, SST_MIX_LOOP0);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_media_l1_controls, SST_MIX_LOOP1);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_media_l2_controls, SST_MIX_LOOP2);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_voip_controls, SST_MIX_VOIP);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_aware_controls, SST_MIX_AWARE);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_vad_controls, SST_MIX_VAD);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_hf_sns_controls, SST_MIX_HF_SNS);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_hf_controls, SST_MIX_HF);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_speech_controls, SST_MIX_SPEECH);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_rxspeech_controls, SST_MIX_RXSPEECH);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_codec0_controls, SST_MIX_CODEC0);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_codec1_controls, SST_MIX_CODEC1);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_bt_controls, SST_MIX_BT);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_fm_controls, SST_MIX_FM);
SST_SBA_DECLARE_MIX_CONTROLS(sst_mix_modem_controls, SST_MIX_MODEM);

static int sst_vb_trigger_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	struct sst_cmd_generic cmd;
	struct sst_data *sst = snd_soc_platform_get_drvdata(w->platform);

	pr_debug("Enter:%s, widget=%s\n", __func__, w->name);
	if (SND_SOC_DAPM_EVENT_ON(event))
		cmd.header.command_id = SBA_VB_START;
	else
		cmd.header.command_id = SBA_IDLE;

	SST_FILL_DEFAULT_DESTINATION(cmd.header.dst);
	cmd.header.length = 0;

	if (SND_SOC_DAPM_EVENT_ON(event))
		sst_dsp->ops->power(true);

	sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
			      SST_TASK_SBA, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);

	if (!SND_SOC_DAPM_EVENT_ON(event))
		sst_dsp->ops->power(false);
	return 0;
}

static void sst_send_slot_map(struct sst_data *sst)
{
	struct sst_param_sba_ssp_slot_map cmd;

	pr_debug("Enter: %s", __func__);

	SST_FILL_DEFAULT_DESTINATION(cmd.header.dst);
	cmd.header.command_id = SBA_SET_SSP_SLOT_MAP;
	cmd.header.length = sizeof(struct sst_param_sba_ssp_slot_map)
				- sizeof(struct sst_dsp_header);

	cmd.param_id = SBA_SET_SSP_SLOT_MAP;
	cmd.param_len = sizeof(cmd.rx_slot_map) + sizeof(cmd.tx_slot_map) + sizeof(cmd.ssp_index);
	cmd.ssp_index = SSP_CODEC;

	memcpy(cmd.rx_slot_map, &sst_ssp_slot_map[0], sizeof(cmd.rx_slot_map));
	memcpy(cmd.tx_slot_map, &sst_ssp_channel_map[0], sizeof(cmd.tx_slot_map));

	sst_fill_and_send_cmd(sst, SST_IPC_IA_SET_PARAMS, SST_FLAG_BLOCKED,
			      SST_TASK_SBA, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);
}

static int sst_ssp_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *k, int event)
{
	struct sst_cmd_sba_hw_set_ssp cmd;
	struct sst_data *sst = snd_soc_platform_get_drvdata(w->platform);
	struct sst_ids *ids = w->priv;
	static int ssp_active[SST_NUM_SSPS];
	unsigned int domain, mux;
	unsigned int ssp_no = ids->ssp->ssp_number;
	int domain_shift, mux_shift;
	const struct sst_ssp_config *config;

	pr_debug("Enter:%s, widget=%s\n", __func__, w->name);

	SST_FILL_DEFAULT_DESTINATION(cmd.header.dst);
	cmd.header.command_id = SBA_HW_SET_SSP;
	cmd.header.length = sizeof(struct sst_cmd_sba_hw_set_ssp)
				- sizeof(struct sst_dsp_header);
	mux_shift = *ids->ssp->mux_shift;
	mux = (mux_shift == -1) ? 0 : get_mux_state(sst, SST_MUX_REG, mux_shift);
	domain_shift = (*ids->ssp->domain_shift)[mux];
	domain = (domain_shift == -1) ? 0 : get_mux_state(sst, SST_MUX_REG, domain_shift);

	config = &(*ids->ssp->ssp_config)[mux][domain];
	pr_debug("%s: ssp_id: %u, mux: %d, domain: %d\n", __func__,
		 config->ssp_id, mux, domain);

	if (SND_SOC_DAPM_EVENT_ON(event))
		ssp_active[ssp_no]++;
	else
		ssp_active[ssp_no]--;

	pr_debug("%s: ssp_no: %u ssp_active: %d", __func__, ssp_no, ssp_active[ssp_no]);
	if (ssp_active[ssp_no])
		cmd.switch_state = SST_SWITCH_ON;
	else
		cmd.switch_state = SST_SWITCH_OFF;

	cmd.selection = config->ssp_id;
	cmd.nb_bits_per_slots = config->bits_per_slot;
	cmd.nb_slots = config->slots;
	cmd.mode = config->ssp_mode | (config->pcm_mode << 1);
	cmd.duplex = config->duplex;
	cmd.active_tx_slot_map = config->active_slot_map;
	cmd.active_rx_slot_map = config->active_slot_map;
	cmd.frame_sync_frequency = config->fs_frequency;
	cmd.frame_sync_polarity = SSP_FS_ACTIVE_HIGH;
	cmd.data_polarity = 1;
	cmd.frame_sync_width = config->fs_width;
	cmd.ssp_protocol = config->ssp_protocol;
	cmd.start_delay = config->start_delay;
	cmd.reserved1 = cmd.reserved2 = 0xFF;

	sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
			      SST_TASK_SBA, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		sst_find_and_send_pipe_algo(w->platform, w);
		sst_send_slot_map(sst);
		sst_set_pipe_gain(ids, sst, 0);
	}
	return 0;
}

static int sst_set_speech_path(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *k, int event)
{
	struct sst_cmd_set_speech_path cmd;
	struct sst_data *sst = snd_soc_platform_get_drvdata(w->platform);
	struct sst_ids *ids = w->priv;
	bool is_wideband;
	static int speech_active;

	pr_debug("%s: widget=%s\n", __func__, w->name);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		speech_active++;
		cmd.switch_state = SST_SWITCH_ON;
	} else {
		speech_active--;
		cmd.switch_state = SST_SWITCH_OFF;
	}

	SST_FILL_DEFAULT_DESTINATION(cmd.header.dst);

	cmd.header.command_id = SBA_VB_SET_SPEECH_PATH;
	cmd.header.length = sizeof(struct sst_cmd_set_speech_path)
				- sizeof(struct sst_dsp_header);
	cmd.config.sample_length = 0;
	cmd.config.rate = 0;		/* 8 khz */
	cmd.config.format = 0;

	is_wideband = get_mux_state(sst, SST_MUX_REG, SST_VOICE_MODE_SHIFT);
	if (is_wideband)
		cmd.config.rate = 1;	/* 16 khz */

	if ((SND_SOC_DAPM_EVENT_ON(event) && (speech_active == 1)) ||
			(SND_SOC_DAPM_EVENT_OFF(event) && (speech_active == 0)))
		sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
				SST_TASK_SBA, 0, &cmd,
				sizeof(cmd.header) + cmd.header.length);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		sst_find_and_send_pipe_algo(w->platform, w);
		sst_set_pipe_gain(ids, sst, 0);
	}

	return 0;

}

static int sst_set_media_path(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *k, int event)
{
	struct sst_cmd_set_media_path cmd;
	struct sst_data *sst = snd_soc_platform_get_drvdata(w->platform);
	struct sst_ids *ids = w->priv;

	pr_debug("%s: widget=%s\n", __func__, w->name);
	pr_debug("%s: task=%u, location=%#x\n", __func__,
				ids->task_id, ids->location_id);

	if (SND_SOC_DAPM_EVENT_ON(event))
		cmd.switch_state = SST_PATH_ON;
	else
		cmd.switch_state = SST_PATH_OFF;

	SST_FILL_DESTINATION(2, cmd.header.dst,
			     ids->location_id, SST_DEFAULT_MODULE_ID);

	/* MMX_SET_MEDIA_PATH == SBA_SET_MEDIA_PATH */
	cmd.header.command_id = MMX_SET_MEDIA_PATH;
	cmd.header.length = sizeof(struct sst_cmd_set_media_path)
				- sizeof(struct sst_dsp_header);

	sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
			      ids->task_id, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		sst_find_and_send_pipe_algo(w->platform, w);
		sst_set_pipe_gain(ids, sst, 0);
	}

	return 0;
}

static int sst_set_media_loop(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	struct sst_cmd_sba_set_media_loop_map cmd;
	struct sst_data *sst = snd_soc_platform_get_drvdata(w->platform);
	struct sst_ids *ids = w->priv;

	pr_debug("Enter:%s, widget=%s\n", __func__, w->name);
	if (SND_SOC_DAPM_EVENT_ON(event))
		cmd.switch_state = SST_SWITCH_ON;
	else
		cmd.switch_state = SST_SWITCH_OFF;

	SST_FILL_DESTINATION(2, cmd.header.dst,
			     ids->location_id, SST_DEFAULT_MODULE_ID);

	cmd.header.command_id = SBA_SET_MEDIA_LOOP_MAP;
	cmd.header.length = sizeof(struct sst_cmd_sba_set_media_loop_map)
				 - sizeof(struct sst_dsp_header);
	cmd.param.part.rate = 2; /* 48khz */

	cmd.param.part.format = ids->format; /* stereo/Mono */
	cmd.param.part.sample_length = 1; /* 24bit left justified*/
	cmd.map = 0; /* Algo sequence: Gain - DRP - FIR - IIR  */

	sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
			      SST_TASK_SBA, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		sst_find_and_send_pipe_algo(w->platform, w);
		sst_set_pipe_gain(ids, sst, 0);
	}
	return 0;
}

static int sst_send_probe_cmd(struct sst_data *sst, u16 probe_pipe_id,
			      int mode, int switch_state,
			      const struct sst_probe_config *probe_cfg)
{
	struct sst_cmd_probe cmd;

	memset(&cmd, 0, sizeof(cmd));

	SST_FILL_DESTINATION(3, cmd.header.dst, SST_DEFAULT_CELL_NBR,
			     probe_pipe_id, SST_DEFAULT_MODULE_ID);
	cmd.header.command_id = SBA_PROBE;
	cmd.header.length = sizeof(struct sst_cmd_probe)
				 - sizeof(struct sst_dsp_header);
	cmd.switch_state = switch_state;

	SST_FILL_DESTINATION(2, cmd.probe_dst,
			     probe_cfg->loc_id, probe_cfg->mod_id);

	cmd.shared_mem = 1;
	cmd.probe_in = 0;
	cmd.probe_out = 0;

	cmd.probe_mode = mode;
	cmd.sample_length = probe_cfg->cfg.s_length;
	cmd.rate = probe_cfg->cfg.rate;
	cmd.format = probe_cfg->cfg.format;
	cmd.sm_buf_id = 1;

	return sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
				     probe_cfg->task_id, 0, &cmd,
				     sizeof(cmd.header) + cmd.header.length);
}

static const struct snd_kcontrol_new sst_probe_controls[];
static const struct sst_probe_config sst_probes[];

#define SST_MAX_PROBE_STREAMS 8
int sst_dpcm_probe_send(struct snd_soc_platform *platform, u16 probe_pipe_id,
			int substream, int direction, bool on)
{
	int switch_state = on ? SST_SWITCH_ON : SST_SWITCH_OFF;
	struct sst_data *sst = snd_soc_platform_get_drvdata(platform);
	const struct sst_probe_config *probe_cfg;
	struct sst_probe_value *probe_val;
	char *type;
	int offset;
	int mode;

	if (direction == SNDRV_PCM_STREAM_CAPTURE) {
		mode = SST_PROBE_EXTRACTOR;
		offset = 0;
		type = "extractor";
	} else {
		mode = SST_PROBE_INJECTOR;
		offset = SST_MAX_PROBE_STREAMS;
		type = "injector";
	}
	/* get the value of the probe connection kcontrol */
	probe_val = (void *)sst_probe_controls[substream + offset].private_value;
	probe_cfg = &sst_probes[probe_val->val];

	pr_debug("%s: substream=%d, direction=%d\n", __func__, substream, direction);
	pr_debug("%s: %s probe point at %s\n", __func__, type, probe_cfg->name);

	return sst_send_probe_cmd(sst, probe_pipe_id, mode, switch_state, probe_cfg);
}

static const struct snd_kcontrol_new sst_mix_sw_aware =
	SOC_SINGLE_EXT("switch", SST_MIX_SWITCH, 0, 1, 0,
		sst_mix_get, sst_mix_put);

static const char * const sst_bt_fm_texts[] = {
	"fm", "bt",
};

static const struct snd_kcontrol_new sst_bt_fm_mux =
	SST_SSP_MUX_CTL("ssp1_out", 0, SST_MUX_REG, SST_BT_FM_MUX_SHIFT, sst_bt_fm_texts,
			sst_mux_get, sst_mux_put);

#define SST_SSP_CODEC_MUX		0
#define SST_SSP_CODEC_DOMAIN		0
#define SST_SSP_MODEM_MUX		0
#define SST_SSP_MODEM_DOMAIN		0
#define SST_SSP_FM_MUX			0
#define SST_SSP_FM_DOMAIN		0
#define SST_SSP_BT_MUX			1
#define SST_SSP_BT_NB_DOMAIN		0
#define SST_SSP_BT_WB_DOMAIN		1

static const int sst_ssp_mux_shift[SST_NUM_SSPS] = {
	[SST_SSP0] = -1,			/* no register shift, i.e. single mux value */
	[SST_SSP1] = SST_BT_FM_MUX_SHIFT,
	[SST_SSP2] = -1,
};

static const int sst_ssp_domain_shift[SST_NUM_SSPS][SST_MAX_SSP_MUX] = {
	[SST_SSP0][0] = -1,			/* no domain shift, i.e. single domain */
	[SST_SSP1] = {
		[SST_SSP_FM_MUX] = -1,
		[SST_SSP_BT_MUX] = SST_BT_MODE_SHIFT,
	},
	[SST_SSP2][0] = -1,
};

static const struct sst_ssp_config
sst_ssp_configs[SST_NUM_SSPS][SST_MAX_SSP_MUX][SST_MAX_SSP_DOMAINS] = {
	[SST_SSP0] = {
		[SST_SSP_MODEM_MUX] = {
			[SST_SSP_MODEM_DOMAIN] = {
				.ssp_id = SSP_MODEM,
				.bits_per_slot = 16,
				.slots = 1,
				.ssp_mode = SSP_MODE_MASTER,
				.pcm_mode = SSP_PCM_MODE_NETWORK,
				.duplex = SSP_DUPLEX,
				.ssp_protocol = SSP_MODE_PCM,
				.fs_width = 1,
				.fs_frequency = SSP_FS_48_KHZ,
				.active_slot_map = 0x1,
				.start_delay = 1,
			},
		},
	},
	[SST_SSP1] = {
		[SST_SSP_FM_MUX] = {
			[SST_SSP_FM_DOMAIN] = {
				.ssp_id = SSP_FM,
				.bits_per_slot = 16,
				.slots = 2,
				.ssp_mode = SSP_MODE_MASTER,
				.pcm_mode = SSP_PCM_MODE_NORMAL,
				.duplex = SSP_DUPLEX,
				.ssp_protocol = SSP_MODE_I2S,
				.fs_width = 32,
				.fs_frequency = SSP_FS_48_KHZ,
				.active_slot_map = 0x3,
				.start_delay = 0,
			},
		},
		[SST_SSP_BT_MUX] = {
			[SST_SSP_BT_NB_DOMAIN] = {
				.ssp_id = SSP_BT,
				.bits_per_slot = 16,
				.slots = 1,
				.ssp_mode = SSP_MODE_MASTER,
				.pcm_mode = SSP_PCM_MODE_NORMAL,
				.duplex = SSP_DUPLEX,
				.ssp_protocol = SSP_MODE_PCM,
				.fs_width = 1,
				.fs_frequency = SSP_FS_8_KHZ,
				.active_slot_map = 0x1,
				.start_delay = 1,
			},
			[SST_SSP_BT_WB_DOMAIN] = {
				.ssp_id = SSP_BT,
				.bits_per_slot = 16,
				.slots = 1,
				.ssp_mode = SSP_MODE_MASTER,
				.pcm_mode = SSP_PCM_MODE_NORMAL,
				.duplex = SSP_DUPLEX,
				.ssp_protocol = SSP_MODE_PCM,
				.fs_width = 1,
				.fs_frequency = SSP_FS_16_KHZ,
				.active_slot_map = 0x1,
				.start_delay = 1,
			},
		},
	},
	[SST_SSP2] = {
		[SST_SSP_CODEC_MUX] = {
			[SST_SSP_CODEC_DOMAIN] = {
				.ssp_id = SSP_CODEC,
				.bits_per_slot = 24,
				.slots = 4,
				.ssp_mode = SSP_MODE_MASTER,
				.pcm_mode = SSP_PCM_MODE_NETWORK,
				.duplex = SSP_DUPLEX,
				.ssp_protocol = SSP_MODE_PCM,
				.fs_width = 1,
				.fs_frequency = SSP_FS_48_KHZ,
				.active_slot_map = 0xF,
				.start_delay = 0,
			},
		},
	},
};

#define SST_SSP_CFG(wssp_no)								\
	(const struct sst_ssp_cfg){ .ssp_config = &sst_ssp_configs[wssp_no],		\
				    .ssp_number = wssp_no,				\
				    .mux_shift = &sst_ssp_mux_shift[wssp_no],		\
				    .domain_shift = &sst_ssp_domain_shift[wssp_no], }

static const struct snd_soc_dapm_widget sst_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("tone"),
	SND_SOC_DAPM_OUTPUT("aware"),
	SND_SOC_DAPM_OUTPUT("vad"),
	SST_SSP_INPUT("modem_in",  sst_ssp_event, SST_SSP_CFG(SST_SSP0)),
	SST_SSP_AIF_IN("codec_in0", sst_ssp_event, SST_SSP_CFG(SST_SSP2)),
	SST_SSP_AIF_IN("codec_in1", sst_ssp_event, SST_SSP_CFG(SST_SSP2)),
	SST_SSP_INPUT("bt_fm_in", sst_ssp_event, SST_SSP_CFG(SST_SSP1)),
	SST_SSP_OUTPUT("modem_out", sst_ssp_event, SST_SSP_CFG(SST_SSP0)),
	SST_SSP_AIF_OUT("codec_out0", sst_ssp_event, SST_SSP_CFG(SST_SSP2)),
	SST_SSP_AIF_OUT("codec_out1", sst_ssp_event, SST_SSP_CFG(SST_SSP2)),
	SST_SSP_OUTPUT("bt_fm_out", sst_ssp_event, SST_SSP_CFG(SST_SSP1)),

	/* Media Paths */
	/* MediaX IN paths are set via ALLOC, so no SET_MEDIA_PATH command */
	SST_PATH_INPUT("media0_in", SST_TASK_MMX, SST_SWM_IN_MEDIA0, NULL),
	SST_PATH_INPUT("media1_in", SST_TASK_MMX, SST_SWM_IN_MEDIA1, NULL),
	SST_PATH_INPUT("media2_in", SST_TASK_MMX, SST_SWM_IN_MEDIA2, sst_set_media_path),
	SST_PATH_INPUT("media3_in", SST_TASK_MMX, SST_SWM_IN_MEDIA3, NULL),
	SST_PATH_OUTPUT("media0_out", SST_TASK_MMX, SST_SWM_OUT_MEDIA0, sst_set_media_path),
	SST_PATH_OUTPUT("media1_out", SST_TASK_MMX, SST_SWM_OUT_MEDIA1, sst_set_media_path),

	/* SBA PCM Paths */
	SST_PATH_INPUT("pcm0_in", SST_TASK_SBA, SST_SWM_IN_PCM0, sst_set_media_path),
	SST_PATH_INPUT("pcm1_in", SST_TASK_SBA, SST_SWM_IN_PCM1, sst_set_media_path),
	SST_PATH_OUTPUT("pcm0_out", SST_TASK_SBA, SST_SWM_OUT_PCM0, sst_set_media_path),
	SST_PATH_OUTPUT("pcm1_out", SST_TASK_SBA, SST_SWM_OUT_PCM1, sst_set_media_path),
	SST_PATH_OUTPUT("pcm2_out", SST_TASK_SBA, SST_SWM_OUT_PCM2, sst_set_media_path),
	/* TODO: check if this needs SET_MEDIA_PATH command*/
	SST_PATH_INPUT("low_pcm0_in", SST_TASK_SBA, SST_SWM_IN_LOW_PCM0, NULL),

	SST_PATH_INPUT("voip_in", SST_TASK_SBA, SST_SWM_IN_VOIP, sst_set_media_path),
	SST_PATH_OUTPUT("voip_out", SST_TASK_SBA, SST_SWM_OUT_VOIP, sst_set_media_path),
	SST_PATH_OUTPUT("aware_out", SST_TASK_SBA, SST_SWM_OUT_AWARE, sst_set_media_path),
	SST_PATH_OUTPUT("vad_out", SST_TASK_SBA, SST_SWM_OUT_VAD, sst_set_media_path),

	/* SBA Loops */
	SST_PATH_INPUT("sprot_loop_in", SST_TASK_SBA, SST_SWM_IN_SPROT_LOOP, NULL),
	SST_PATH_INPUT("media_loop1_in", SST_TASK_SBA, SST_SWM_IN_MEDIA_LOOP1, NULL),
	SST_PATH_INPUT("media_loop2_in", SST_TASK_SBA, SST_SWM_IN_MEDIA_LOOP2, NULL),
	SST_PATH_MEDIA_LOOP_OUTPUT("sprot_loop_out", SST_TASK_SBA, SST_SWM_OUT_SPROT_LOOP, SST_FMT_MONO, sst_set_media_loop),
	SST_PATH_MEDIA_LOOP_OUTPUT("media_loop1_out", SST_TASK_SBA, SST_SWM_OUT_MEDIA_LOOP1, SST_FMT_MONO, sst_set_media_loop),
	SST_PATH_MEDIA_LOOP_OUTPUT("media_loop2_out", SST_TASK_SBA, SST_SWM_OUT_MEDIA_LOOP2, SST_FMT_STEREO, sst_set_media_loop),

	/* TODO: need to send command */
	SST_PATH_INPUT("sidetone_in", SST_TASK_SBA, SST_SWM_IN_SIDETONE, NULL),
	SST_PATH_INPUT("tone_in", SST_TASK_SBA, SST_SWM_IN_TONE, NULL),
	SST_PATH_INPUT("bt_in", SST_TASK_SBA, SST_SWM_IN_BT, NULL),
	SST_PATH_INPUT("fm_in", SST_TASK_SBA, SST_SWM_IN_FM, NULL),
	SST_PATH_OUTPUT("bt_out", SST_TASK_SBA, SST_SWM_OUT_BT, NULL),
	SST_PATH_OUTPUT("fm_out", SST_TASK_SBA, SST_SWM_OUT_FM, NULL),

	/* SBA Voice Paths */
	SST_PATH_INPUT("speech_in", SST_TASK_SBA, SST_SWM_IN_SPEECH, sst_set_speech_path),
	SST_PATH_INPUT("txspeech_in", SST_TASK_SBA, SST_SWM_IN_TXSPEECH, sst_set_speech_path),
	SST_PATH_OUTPUT("hf_sns_out", SST_TASK_SBA, SST_SWM_OUT_HF_SNS, sst_set_speech_path),
	SST_PATH_OUTPUT("hf_out", SST_TASK_SBA, SST_SWM_OUT_HF, sst_set_speech_path),
	SST_PATH_OUTPUT("speech_out", SST_TASK_SBA, SST_SWM_OUT_SPEECH, sst_set_speech_path),
	SST_PATH_OUTPUT("rxspeech_out", SST_TASK_SBA, SST_SWM_OUT_RXSPEECH, sst_set_speech_path),

	/* Media Mixers */
	SST_SWM_MIXER("media0_out mix 0", SST_MIX_MEDIA0, SST_TASK_MMX, SST_SWM_OUT_MEDIA0,
		      sst_mix_media0_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("media1_out mix 0", SST_MIX_MEDIA1, SST_TASK_MMX, SST_SWM_OUT_MEDIA1,
		      sst_mix_media1_controls, sst_swm_mixer_event),

	/* SBA PCM mixers */
	SST_SWM_MIXER("pcm0_out mix 0", SST_MIX_PCM0, SST_TASK_SBA, SST_SWM_OUT_PCM0,
		      sst_mix_pcm0_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("pcm1_out mix 0", SST_MIX_PCM1, SST_TASK_SBA, SST_SWM_OUT_PCM1,
		      sst_mix_pcm1_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("pcm2_out mix 0", SST_MIX_PCM2, SST_TASK_SBA, SST_SWM_OUT_PCM2,
		      sst_mix_pcm2_controls, sst_swm_mixer_event),

	/* SBA Loop mixers */
	SST_SWM_MIXER("sprot_loop_out mix 0", SST_MIX_LOOP0, SST_TASK_SBA, SST_SWM_OUT_SPROT_LOOP,
		      sst_mix_sprot_l0_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("media_loop1_out mix 0", SST_MIX_LOOP1, SST_TASK_SBA, SST_SWM_OUT_MEDIA_LOOP1,
		      sst_mix_media_l1_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("media_loop2_out mix 0", SST_MIX_LOOP2, SST_TASK_SBA, SST_SWM_OUT_MEDIA_LOOP2,
		      sst_mix_media_l2_controls, sst_swm_mixer_event),

	SST_SWM_MIXER("voip_out mix 0", SST_MIX_VOIP, SST_TASK_SBA, SST_SWM_OUT_VOIP,
		      sst_mix_voip_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("aware_out mix 0", SST_MIX_AWARE, SST_TASK_SBA, SST_SWM_OUT_AWARE,
		      sst_mix_aware_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("vad_out mix 0", SST_MIX_VAD, SST_TASK_SBA, SST_SWM_OUT_VAD,
		      sst_mix_vad_controls, sst_swm_mixer_event),

	/* SBA Voice mixers */
	SST_SWM_MIXER("hf_sns_out mix 0", SST_MIX_HF_SNS, SST_TASK_SBA, SST_SWM_OUT_HF_SNS,
		      sst_mix_hf_sns_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("hf_out mix 0", SST_MIX_HF, SST_TASK_SBA, SST_SWM_OUT_HF,
		      sst_mix_hf_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("speech_out mix 0", SST_MIX_SPEECH, SST_TASK_SBA, SST_SWM_OUT_SPEECH,
		      sst_mix_speech_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("rxspeech_out mix 0", SST_MIX_RXSPEECH, SST_TASK_SBA, SST_SWM_OUT_RXSPEECH,
		      sst_mix_rxspeech_controls, sst_swm_mixer_event),

	/* SBA Backend mixers */
	SST_SWM_MIXER("codec_out0 mix 0", SST_MIX_CODEC0, SST_TASK_SBA, SST_SWM_OUT_CODEC0,
		      sst_mix_codec0_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("codec_out1 mix 0", SST_MIX_CODEC1, SST_TASK_SBA, SST_SWM_OUT_CODEC1,
		      sst_mix_codec1_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("bt_out mix 0", SST_MIX_BT, SST_TASK_SBA, SST_SWM_OUT_BT,
		      sst_mix_bt_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("fm_out mix 0", SST_MIX_FM, SST_TASK_SBA, SST_SWM_OUT_FM,
		      sst_mix_fm_controls, sst_swm_mixer_event),
	SST_SWM_MIXER("modem_out mix 0", SST_MIX_MODEM, SST_TASK_SBA, SST_SWM_OUT_MODEM,
		      sst_mix_modem_controls, sst_swm_mixer_event),

	SND_SOC_DAPM_SWITCH("aware_out aware 0", SND_SOC_NOPM, 0, 0, &sst_mix_sw_aware),
	SND_SOC_DAPM_MUX("ssp1_out mux 0", SND_SOC_NOPM, 0, 0, &sst_bt_fm_mux),

	SND_SOC_DAPM_SUPPLY("VBTimer", SND_SOC_NOPM, 0, 0,
			    sst_vb_trigger_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route intercon[] = {
	{"media0_in", NULL, "Compress Playback"},
	{"media1_in", NULL, "Headset Playback"},
	{"media2_in", NULL, "pcm0_out"},
	{"media3_in", NULL, "Deepbuffer Playback"},

	{"media0_out mix 0", "media0_in", "media0_in"},
	{"media0_out mix 0", "media1_in", "media1_in"},
	{"media0_out mix 0", "media2_in", "media2_in"},
	{"media0_out mix 0", "media3_in", "media3_in"},
	{"media1_out mix 0", "media0_in", "media0_in"},
	{"media1_out mix 0", "media1_in", "media1_in"},
	{"media1_out mix 0", "media2_in", "media2_in"},
	{"media1_out mix 0", "media3_in", "media3_in"},

	{"media0_out", NULL, "media0_out mix 0"},
	{"media1_out", NULL, "media1_out mix 0"},
	{"pcm0_in", NULL, "media0_out"},
	{"pcm1_in", NULL, "media1_out"},

	{"Headset Capture", NULL, "pcm1_out"},
	{"Headset Capture", NULL, "pcm2_out"},
	{"pcm0_out", NULL, "pcm0_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("pcm0_out mix 0"),
	{"pcm1_out", NULL, "pcm1_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("pcm1_out mix 0"),
	{"pcm2_out", NULL, "pcm2_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("pcm2_out mix 0"),

	{"media_loop1_in", NULL, "media_loop1_out"},
	{"media_loop1_out", NULL, "media_loop1_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("media_loop1_out mix 0"),
	{"media_loop2_in", NULL, "media_loop2_out"},
	{"media_loop2_out", NULL, "media_loop2_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("media_loop2_out mix 0"),
	{"sprot_loop_in", NULL, "sprot_loop_out"},
	{"sprot_loop_out", NULL, "sprot_loop_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("sprot_loop_out mix 0"),

	{"voip_in", NULL, "VOIP Playback"},
	{"VOIP Capture", NULL, "voip_out"},
	{"voip_out", NULL, "voip_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("voip_out mix 0"),

	{"aware", NULL, "aware_out"},
	{"aware_out", NULL, "aware_out aware 0"},
	{"aware_out aware 0", "switch", "aware_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("aware_out mix 0"),
	{"vad", NULL, "vad_out"},
	{"vad_out", NULL, "vad_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("vad_out mix 0"),

	{"codec_out0", NULL, "codec_out0 mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("codec_out0 mix 0"),
	{"codec_out1", NULL, "codec_out1 mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("codec_out1 mix 0"),
	{"modem_out", NULL, "modem_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("modem_out mix 0"),

	{"bt_fm_out", NULL, "ssp1_out mux 0"},
	{"ssp1_out mux 0", "bt", "bt_out"},
	{"ssp1_out mux 0", "fm", "fm_out"},
	{"bt_out", NULL, "bt_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("bt_out mix 0"),
	{"fm_out", NULL, "fm_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("fm_out mix 0"),
	{"bt_in", NULL, "bt_fm_in"},
	{"fm_in", NULL, "bt_fm_in"},

	/* Uplink processing */
	{"txspeech_in", NULL, "hf_sns_out"},
	{"txspeech_in", NULL, "hf_out"},
	{"txspeech_in", NULL, "speech_out"},

	{"hf_sns_out", NULL, "hf_sns_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("hf_sns_out mix 0"),
	{"hf_out", NULL, "hf_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("hf_out mix 0"),
	{"speech_out", NULL, "speech_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("speech_out mix 0"),

	/* Downlink processing */
	{"speech_in", NULL, "rxspeech_out"},
	{"rxspeech_out", NULL, "rxspeech_out mix 0"},
	SST_SBA_MIXER_GRAPH_MAP("rxspeech_out mix 0"),

	/* TODO: add Tone inputs */
	/* TODO: add Low Latency stream support */

	{"Headset Capture", NULL, "VBTimer"},
	{"Headset Playback", NULL, "VBTimer"},
	{"Deepbuffer Playback", NULL, "VBTimer"},
	{"Compress Playback", NULL, "VBTimer"},
	{"VOIP Playback", NULL, "VBTimer"},
	{"aware", NULL, "VBTimer"},
	{"modem_in", NULL, "VBTimer"},
	{"modem_out", NULL, "VBTimer"},
	{"bt_fm_in", NULL, "VBTimer"},
	{"bt_fm_out", NULL, "VBTimer"},
};

static const char * const sst_nb_wb_texts[] = {
	"narrowband", "wideband",
};

static const struct snd_kcontrol_new sst_mux_controls[] = {
	SST_SSP_MUX_CTL("domain voice mode", 0, SST_MUX_REG, SST_VOICE_MODE_SHIFT, sst_nb_wb_texts,
			sst_mode_get, sst_mode_put),
	SST_SSP_MUX_CTL("domain bt mode", 0, SST_MUX_REG, SST_BT_MODE_SHIFT, sst_nb_wb_texts,
			sst_mode_get, sst_mode_put),
};

static const char * const slot_names[] = {
	"none",
	"slot 0", "slot 1", "slot 2", "slot 3",
	"slot 4", "slot 5", "slot 6", "slot 7", /* not supported by FW */
};

static const char * const channel_names[] = {
	"none",
	"codec_out0_0", "codec_out0_1", "codec_out1_0", "codec_out1_1",
	"codec_out2_0", "codec_out2_1", "codec_out3_0", "codec_out3_1", /* not supported by FW */
};

#define SST_INTERLEAVER(xpname, slot_name, slotno) \
	SST_SSP_SLOT_CTL(xpname, "interleaver", slot_name, slotno, 1, \
			 channel_names, sst_slot_get, sst_slot_put)

#define SST_DEINTERLEAVER(xpname, channel_name, channel_no) \
	SST_SSP_SLOT_CTL(xpname, "deinterleaver", channel_name, channel_no, 0, \
			 slot_names, sst_slot_get, sst_slot_put)

static const struct snd_kcontrol_new sst_slot_controls[] = {
	SST_INTERLEAVER("codec_out", "slot 0", 0),
	SST_INTERLEAVER("codec_out", "slot 1", 1),
	SST_INTERLEAVER("codec_out", "slot 2", 2),
	SST_INTERLEAVER("codec_out", "slot 3", 3),
	SST_DEINTERLEAVER("codec_in", "codec_in0_0", 0),
	SST_DEINTERLEAVER("codec_in", "codec_in0_1", 1),
	SST_DEINTERLEAVER("codec_in", "codec_in1_0", 2),
	SST_DEINTERLEAVER("codec_in", "codec_in1_1", 3),
};

#define SST_NUM_PROBE_CONNECTION_PTS 31
static const struct sst_probe_config sst_probes[SST_NUM_PROBE_CONNECTION_PTS] = {
	/* TODO: get this struct from FW config data */
	/* TODO: only gain outputs supported currently */
	{ "media0_in gain", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media1_in gain", SST_PATH_INDEX_MEDIA1_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media2_in gain", SST_PATH_INDEX_MEDIA2_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media3_in gain", SST_PATH_INDEX_MEDIA3_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_MMX, { 1, 2, 1 } },
	{ "pcm0_in gain", SST_PATH_INDEX_PCM0_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "pcm1_in gain", SST_PATH_INDEX_PCM1_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "pcm1_out gain", SST_PATH_INDEX_PCM1_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "pcm2_out gain", SST_PATH_INDEX_PCM2_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "voip_in gain", SST_PATH_INDEX_VOIP_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "voip_out gain", SST_PATH_INDEX_VOIP_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "aware_out gain", SST_PATH_INDEX_AWARE_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "vad_out gain", SST_PATH_INDEX_VAD_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_sns_out gain", SST_PATH_INDEX_HF_SNS_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_out gain", SST_PATH_INDEX_HF_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "speech_out gain", SST_PATH_INDEX_SPEECH_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "txspeech_in gain", SST_PATH_INDEX_TX_SPEECH_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "rxspeech_out gain", SST_PATH_INDEX_RX_SPEECH_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "speech_in gain", SST_PATH_INDEX_SPEECH_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "media_loop1_out gain", SST_PATH_INDEX_MEDIA_LOOP1_OUT , SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "media_loop2_out gain", SST_PATH_INDEX_MEDIA_LOOP2_OUT , SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "tone_in gain", SST_PATH_INDEX_TONE_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_out0 gain", SST_PATH_INDEX_CODEC_OUT0, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_out1 gain", SST_PATH_INDEX_CODEC_OUT1, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "bt_out gain", SST_PATH_INDEX_BT_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "fm_out gain", SST_PATH_INDEX_FM_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "modem_out gain", SST_PATH_INDEX_MODEM_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_in0 gain", SST_PATH_INDEX_CODEC_IN0, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_in1 gain", SST_PATH_INDEX_CODEC_IN1, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "bt_in gain", SST_PATH_INDEX_BT_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "fm_in gain", SST_PATH_INDEX_FM_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "modem_in gain", SST_PATH_INDEX_MODEM_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
};

/* initialized based on names in sst_probes array */
static const char *sst_probe_enum_texts[SST_NUM_PROBE_CONNECTION_PTS];
static const SOC_ENUM_SINGLE_EXT_DECL(sst_probe_enum, sst_probe_enum_texts);

#define SST_PROBE_CTL(name, num)						\
	SST_PROBE_ENUM(SST_PROBE_CTL_NAME(name, num, "connection"),		\
		       sst_probe_enum, sst_probe_get, sst_probe_put)
	/* TODO: implement probe gains
	SOC_SINGLE_EXT_TLV(SST_PROBE_CTL_NAME(name, num, "gains"), xreg, xshift,
		xmax, xinv, xget, xput, sst_gain_tlv_common)
	*/

static const struct snd_kcontrol_new sst_probe_controls[] = {
	SST_PROBE_CTL("probe out", 0),
	SST_PROBE_CTL("probe out", 1),
	SST_PROBE_CTL("probe out", 2),
	SST_PROBE_CTL("probe out", 3),
	SST_PROBE_CTL("probe out", 4),
	SST_PROBE_CTL("probe out", 5),
	SST_PROBE_CTL("probe out", 6),
	SST_PROBE_CTL("probe out", 7),
	SST_PROBE_CTL("probe in", 0),
	SST_PROBE_CTL("probe in", 1),
	SST_PROBE_CTL("probe in", 2),
	SST_PROBE_CTL("probe in", 3),
	SST_PROBE_CTL("probe in", 4),
	SST_PROBE_CTL("probe in", 5),
	SST_PROBE_CTL("probe in", 6),
	SST_PROBE_CTL("probe in", 7),
};

/* Gain helper with min/max set */
#define SST_GAIN(name, path_id, task_id, instance, gain_var)				\
	SST_GAIN_KCONTROLS(name, "gain", SST_GAIN_MIN_VALUE, SST_GAIN_MAX_VALUE,	\
		SST_GAIN_TC_MIN, SST_GAIN_TC_MAX,					\
		sst_gain_get, sst_gain_put,						\
		SST_MODULE_ID_GAIN_CELL, path_id, instance, task_id,			\
		sst_gain_tlv_common, gain_var)

#define SST_VOLUME(name, path_id, task_id, instance, gain_var)				\
	SST_GAIN_KCONTROLS(name, "volume", SST_GAIN_MIN_VALUE, SST_GAIN_MAX_VALUE,	\
		SST_GAIN_TC_MIN, SST_GAIN_TC_MAX,					\
		sst_gain_get, sst_gain_put,						\
		SST_MODULE_ID_VOLUME, path_id, instance, task_id,			\
		sst_gain_tlv_common, gain_var)

#define SST_NUM_GAINS 36
static struct sst_gain_value sst_gains[SST_NUM_GAINS];

static const struct snd_kcontrol_new sst_gain_controls[] = {
	SST_GAIN("media0_in", SST_PATH_INDEX_MEDIA0_IN, SST_TASK_MMX, 0, &sst_gains[0]),
	SST_GAIN("media1_in", SST_PATH_INDEX_MEDIA1_IN, SST_TASK_MMX, 0, &sst_gains[1]),
	SST_GAIN("media2_in", SST_PATH_INDEX_MEDIA2_IN, SST_TASK_MMX, 0, &sst_gains[2]),
	SST_GAIN("media3_in", SST_PATH_INDEX_MEDIA3_IN, SST_TASK_MMX, 0, &sst_gains[3]),

	SST_GAIN("pcm0_in", SST_PATH_INDEX_PCM0_IN, SST_TASK_SBA, 0, &sst_gains[4]),
	SST_GAIN("pcm1_in", SST_PATH_INDEX_PCM1_IN, SST_TASK_SBA, 0, &sst_gains[5]),
	SST_GAIN("low_pcm0_in", SST_PATH_INDEX_LOW_PCM0_IN, SST_TASK_SBA, 0, &sst_gains[6]),
	SST_GAIN("pcm1_out", SST_PATH_INDEX_PCM1_OUT, SST_TASK_SBA, 0, &sst_gains[7]),
	SST_GAIN("pcm2_out", SST_PATH_INDEX_PCM1_OUT, SST_TASK_SBA, 0, &sst_gains[8]),

	SST_GAIN("voip_in", SST_PATH_INDEX_VOIP_IN, SST_TASK_SBA, 0, &sst_gains[9]),
	SST_GAIN("voip_out", SST_PATH_INDEX_VOIP_OUT, SST_TASK_SBA, 0, &sst_gains[10]),
	SST_GAIN("tone_in", SST_PATH_INDEX_TONE_IN, SST_TASK_SBA, 0, &sst_gains[11]),

	SST_GAIN("aware_out", SST_PATH_INDEX_AWARE_OUT, SST_TASK_SBA, 0, &sst_gains[12]),
	SST_GAIN("vad_out", SST_PATH_INDEX_VAD_OUT, SST_TASK_SBA, 0, &sst_gains[13]),

	SST_GAIN("hf_sns_out", SST_PATH_INDEX_HF_SNS_OUT, SST_TASK_SBA, 0, &sst_gains[14]),
	SST_GAIN("hf_out", SST_PATH_INDEX_HF_OUT, SST_TASK_SBA, 0, &sst_gains[15]),
	SST_GAIN("speech_out", SST_PATH_INDEX_SPEECH_OUT, SST_TASK_SBA, 0, &sst_gains[16]),
	SST_GAIN("txspeech_in", SST_PATH_INDEX_TX_SPEECH_IN, SST_TASK_SBA, 0, &sst_gains[17]),
	SST_GAIN("rxspeech_out", SST_PATH_INDEX_RX_SPEECH_OUT, SST_TASK_SBA, 0, &sst_gains[18]),
	SST_GAIN("speech_in", SST_PATH_INDEX_SPEECH_IN, SST_TASK_SBA, 0, &sst_gains[19]),

	SST_GAIN("codec_in0", SST_PATH_INDEX_CODEC_IN0, SST_TASK_SBA, 0, &sst_gains[20]),
	SST_GAIN("codec_in1", SST_PATH_INDEX_CODEC_IN1, SST_TASK_SBA, 0, &sst_gains[21]),
	SST_GAIN("codec_out0", SST_PATH_INDEX_CODEC_OUT0, SST_TASK_SBA, 0, &sst_gains[22]),
	SST_GAIN("codec_out1", SST_PATH_INDEX_CODEC_OUT1, SST_TASK_SBA, 0, &sst_gains[23]),
	SST_GAIN("bt_out", SST_PATH_INDEX_BT_OUT, SST_TASK_SBA, 0, &sst_gains[24]),
	SST_GAIN("fm_out", SST_PATH_INDEX_FM_OUT, SST_TASK_SBA, 0, &sst_gains[25]),
	SST_GAIN("bt_in", SST_PATH_INDEX_BT_IN, SST_TASK_SBA, 0, &sst_gains[26]),
	SST_GAIN("fm_in", SST_PATH_INDEX_FM_IN, SST_TASK_SBA, 0, &sst_gains[27]),
	SST_GAIN("modem_in", SST_PATH_INDEX_MODEM_IN, SST_TASK_SBA, 0, &sst_gains[28]),
	SST_GAIN("modem_out", SST_PATH_INDEX_MODEM_OUT, SST_TASK_SBA, 0, &sst_gains[29]),
	SST_GAIN("media_loop1_out", SST_PATH_INDEX_MEDIA_LOOP1_OUT, SST_TASK_SBA, 0, &sst_gains[30]),
	SST_GAIN("media_loop2_out", SST_PATH_INDEX_MEDIA_LOOP2_OUT, SST_TASK_SBA, 0, &sst_gains[31]),
	SST_GAIN("sprot_loop_out", SST_PATH_INDEX_SPROT_LOOP_OUT, SST_TASK_SBA, 0, &sst_gains[32]),
	SST_VOLUME("media0_in", SST_PATH_INDEX_MEDIA0_IN, SST_TASK_MMX, 0, &sst_gains[33]),
	SST_GAIN("sidetone_in", SST_PATH_INDEX_SIDETONE_IN, SST_TASK_SBA, 0, &sst_gains[34]),
	SST_GAIN("speech_out", SST_PATH_INDEX_SPEECH_OUT, SST_TASK_FBA_UL, 1, &sst_gains[35]),
};

static const struct snd_kcontrol_new sst_algo_controls[] = {
	SST_ALGO_KCONTROL_BYTES("media_loop1_out", "fir", 138, SST_MODULE_ID_FIR_24,
		 SST_PATH_INDEX_MEDIA_LOOP1_OUT, 0, SST_TASK_SBA, SBA_VB_SET_FIR),
	SST_ALGO_KCONTROL_BYTES("media_loop1_out", "iir", 300, SST_MODULE_ID_IIR_24,
		SST_PATH_INDEX_MEDIA_LOOP1_OUT, 0, SST_TASK_SBA, SBA_VB_SET_IIR),
	SST_ALGO_KCONTROL_BYTES("media_loop1_out", "mdrp", 76, SST_MODULE_ID_MDRP,
		SST_PATH_INDEX_MEDIA_LOOP1_OUT, 0, SST_TASK_SBA, SBA_SET_MDRP),
	SST_ALGO_KCONTROL_BYTES("media_loop2_out", "fir", 272, SST_MODULE_ID_FIR_24,
		SST_PATH_INDEX_MEDIA_LOOP2_OUT, 0, SST_TASK_SBA, SBA_VB_SET_FIR),
	SST_ALGO_KCONTROL_BYTES("media_loop2_out", "iir", 300, SST_MODULE_ID_IIR_24,
		SST_PATH_INDEX_MEDIA_LOOP2_OUT, 0, SST_TASK_SBA, SBA_VB_SET_IIR),
	SST_ALGO_KCONTROL_BYTES("media_loop2_out", "mdrp", 76, SST_MODULE_ID_MDRP,
		SST_PATH_INDEX_MEDIA_LOOP2_OUT, 0, SST_TASK_SBA, SBA_SET_MDRP),
	SST_ALGO_KCONTROL_BYTES("aware_out", "fir", 272, SST_MODULE_ID_FIR_24,
		SST_PATH_INDEX_AWARE_OUT, 0, SST_TASK_SBA, SBA_VB_SET_FIR),
	SST_ALGO_KCONTROL_BYTES("aware_out", "iir", 300, SST_MODULE_ID_IIR_24,
		SST_PATH_INDEX_AWARE_OUT, 0, SST_TASK_SBA, SBA_VB_SET_IIR),
	SST_ALGO_KCONTROL_BYTES("aware_out", "aware", 48, SST_MODULE_ID_CONTEXT_ALGO_AWARE,
		SST_PATH_INDEX_AWARE_OUT, 0, SST_TASK_AWARE, AWARE_ENV_CLASS_PARAMS),
	SST_ALGO_KCONTROL_BYTES("vad_out", "fir", 272, SST_MODULE_ID_FIR_24,
		SST_PATH_INDEX_VAD_OUT, 0, SST_TASK_SBA, SBA_VB_SET_FIR),
	SST_ALGO_KCONTROL_BYTES("vad_out", "iir", 300, SST_MODULE_ID_IIR_24,
		SST_PATH_INDEX_VAD_OUT, 0, SST_TASK_SBA, SBA_VB_SET_IIR),
	SST_ALGO_KCONTROL_BYTES("sprot_loop_out", "lpro", 192, SST_MODULE_ID_SPROT,
		SST_PATH_INDEX_SPROT_LOOP_OUT, 0, SST_TASK_SBA, SBA_VB_LPRO),
	SST_ALGO_KCONTROL_BYTES("codec_in0", "dcr", 300, SST_MODULE_ID_FILT_DCR,
		SST_PATH_INDEX_CODEC_IN0, 0, SST_TASK_SBA, SBA_VB_SET_IIR),
	SST_ALGO_KCONTROL_BYTES("codec_in1", "dcr", 300, SST_MODULE_ID_FILT_DCR,
		SST_PATH_INDEX_CODEC_IN1, 0, SST_TASK_SBA, SBA_VB_SET_IIR),
	/* Uplink */
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "fir_speech", 136, SST_MODULE_ID_FIR_16,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_SET_FIR),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "fir_hf_sns", 136, SST_MODULE_ID_FIR_16,
		SST_PATH_INDEX_HF_SNS_OUT, 0, SST_TASK_FBA_UL, FBA_VB_SET_FIR),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "iir_speech", 48, SST_MODULE_ID_IIR_16,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_SET_IIR),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "iir_hf_sns", 48, SST_MODULE_ID_IIR_16,
		SST_PATH_INDEX_HF_SNS_OUT, 0, SST_TASK_FBA_UL, FBA_VB_SET_IIR),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "aec", 640, SST_MODULE_ID_AEC,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_AEC),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "nr", 38, SST_MODULE_ID_NR,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_NR_UL),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "agc", 58, SST_MODULE_ID_AGC,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_AGC),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "biquad", 22, SST_MODULE_ID_DRP,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_SET_BIQUAD_D_C),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "compr", 36, SST_MODULE_ID_DRP,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_DUAL_BAND_COMP),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "sns", 324, SST_MODULE_ID_NR_SNS,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_SNS),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "ser", 42, SST_MODULE_ID_SER,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_SER),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "cni", 48, SST_MODULE_ID_CNI_TX,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_TX_CNI),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "ref", 24, SST_MODULE_ID_REF_LINE,
		SST_PATH_INDEX_HF_OUT, 0, SST_TASK_FBA_UL, FBA_VB_SET_REF_LINE),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "delay", 6, SST_MODULE_ID_EDL,
		SST_PATH_INDEX_HF_OUT, 0, SST_TASK_FBA_UL, FBA_VB_SET_DELAY_LINE),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "bmf", 264, SST_MODULE_ID_BMF,
		SST_PATH_INDEX_HF_SNS_OUT, 0, SST_TASK_FBA_UL, FBA_VB_BMF),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_out", "ul_module", "dnr", 18, SST_MODULE_ID_DNR,
		SST_PATH_INDEX_SPEECH_OUT, 0, SST_TASK_FBA_UL, FBA_VB_DNR),
	/* Downlink */
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_in", "dl_module", "ana", 52, SST_MODULE_ID_ANA,
		SST_PATH_INDEX_SPEECH_IN, 0, SST_TASK_FBA_DL, FBA_VB_ANA),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_in", "dl_module", "fir", 136, SST_MODULE_ID_FIR_16,
		SST_PATH_INDEX_SPEECH_IN, 0, SST_TASK_FBA_DL, FBA_VB_SET_FIR),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_in", "dl_module", "iir", 48, SST_MODULE_ID_IIR_16,
		SST_PATH_INDEX_SPEECH_IN, 0, SST_TASK_FBA_DL, FBA_VB_SET_IIR),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_in", "dl_module", "nr", 38, SST_MODULE_ID_NR,
		SST_PATH_INDEX_SPEECH_IN, 0, SST_TASK_FBA_DL, FBA_VB_NR_DL),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_in", "dl_module", "biquad", 22, SST_MODULE_ID_DRP,
		SST_PATH_INDEX_SPEECH_IN, 0, SST_TASK_FBA_DL, FBA_VB_SET_BIQUAD_D_C),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_in", "dl_module", "compr", 36, SST_MODULE_ID_DRP,
		SST_PATH_INDEX_SPEECH_IN, 0, SST_TASK_FBA_DL, FBA_VB_DUAL_BAND_COMP),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_in", "dl_module", "cni", 48, SST_MODULE_ID_CNI,
		SST_PATH_INDEX_SPEECH_IN, 0, SST_TASK_FBA_DL, FBA_VB_RX_CNI),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_in", "dl_module", "bwx", 54, SST_MODULE_ID_BWX,
		SST_PATH_INDEX_SPEECH_IN, 0, SST_TASK_FBA_DL, FBA_VB_BWX),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_in", "dl_module", "gmm", 586, SST_MODULE_ID_BWX,
		SST_PATH_INDEX_SPEECH_IN, 0, SST_TASK_FBA_DL, FBA_VB_GMM),
	SST_COMBO_ALGO_KCONTROL_BYTES("speech_in", "dl_module", "glc", 18, SST_MODULE_ID_GLC,
		SST_PATH_INDEX_SPEECH_IN, 0, SST_TASK_FBA_DL, FBA_VB_GLC),
};

static const struct snd_kcontrol_new sst_debug_controls[] = {
	SND_SOC_BYTES_EXT("sst debug byte control", SST_MAX_BIN_BYTES,
		       sst_byte_control_get, sst_byte_control_set),
};

static inline bool is_sst_dapm_widget(struct snd_soc_dapm_widget *w)
{
	if ((w->id == snd_soc_dapm_pga) ||
	    (w->id == snd_soc_dapm_aif_in) ||
	    (w->id == snd_soc_dapm_aif_out) ||
	    (w->id == snd_soc_dapm_input) ||
	    (w->id == snd_soc_dapm_output) ||
	    (w->id == snd_soc_dapm_mixer))
		return true;
	else
		return false;
}

int sst_send_pipe_gains(struct snd_soc_dai *dai, int stream, int mute)
{
	struct snd_soc_platform *platform = dai->platform;
	struct sst_data *sst = snd_soc_platform_get_drvdata(platform);
	struct snd_soc_dapm_widget *w;
	struct snd_soc_dapm_path *p = NULL;

	pr_debug("%s: enter, dai-name=%s dir=%d\n", __func__, dai->name, stream);

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_debug("Stream name=%s\n", dai->playback_widget->name);
		w = dai->playback_widget;
		list_for_each_entry(p, &w->sinks, list_source) {
			if (p->connected && !p->connected(w, p->sink))
				continue;

			if (p->connect && p->sink->power && is_sst_dapm_widget(p->sink)) {
				struct sst_ids *ids = p->sink->priv;

				pr_debug("send gains for widget=%s\n", p->sink->name);
				sst_set_pipe_gain(ids, sst, mute);
			}
		}
	} else {
		pr_debug("Stream name=%s\n", dai->capture_widget->name);
		w = dai->capture_widget;
		list_for_each_entry(p, &w->sources, list_sink) {
			if (p->connected && !p->connected(w, p->sink))
				continue;

			if (p->connect &&  p->source->power && is_sst_dapm_widget(p->source)) {
				struct sst_ids *ids = p->source->priv;

				pr_debug("send gain for widget=%s\n", p->source->name);
				sst_set_pipe_gain(ids, sst, mute);
			}
		}
	}
	return 0;
}

static int sst_fill_module_list(struct snd_kcontrol *kctl,
	 struct snd_soc_dapm_widget *w, int type)
{
	struct module *module = NULL;
	struct sst_ids *ids = w->priv;

	module = devm_kzalloc(w->platform->dev, sizeof(*module), GFP_KERNEL);
	if (!module) {
		pr_err("kzalloc block failed\n");
		return -ENOMEM;
	}

	if (type == SST_MODULE_GAIN) {
		struct sst_gain_mixer_control *mc = (void *)kctl->private_value;

		mc->w = w;
		module->kctl = kctl;
		list_add_tail(&module->node, &ids->gain_list);
	} else if (type == SST_MODULE_ALGO) {
		struct sst_algo_control *bc = (void *)kctl->private_value;

		bc->w = w;
		module->kctl = kctl;
		list_add_tail(&module->node, &ids->algo_list);
	}

	return 0;
}

static int sst_fill_widget_module_info(struct snd_soc_dapm_widget *w,
	struct snd_soc_platform *platform)
{
	struct snd_kcontrol *kctl;
	int index, ret = 0;
	struct snd_card *card = platform->card->snd_card;
	char *idx;

	down_read(&card->controls_rwsem);

	list_for_each_entry(kctl, &card->controls, list) {
		idx = strstr(kctl->id.name, " ");
		if (idx == NULL)
			continue;
		index  = strlen(kctl->id.name) - strlen(idx);
		if (strstr(kctl->id.name, "volume") &&
		    !strncmp(kctl->id.name, w->name, index))
			ret = sst_fill_module_list(kctl, w, SST_MODULE_GAIN);
		else if (strstr(kctl->id.name, "params") &&
			 !strncmp(kctl->id.name, w->name, index))
			ret = sst_fill_module_list(kctl, w, SST_MODULE_ALGO);
		else if (strstr(kctl->id.name, "mute") &&
			 !strncmp(kctl->id.name, w->name, index)) {
			struct sst_gain_mixer_control *mc = (void *)kctl->private_value;
			mc->w = w;
		}
		if (ret < 0) {
			up_read(&card->controls_rwsem);
			return ret;
		}
	}
	up_read(&card->controls_rwsem);
	return 0;
}

static int sst_map_modules_to_pipe(struct snd_soc_platform *platform)
{
	struct snd_soc_dapm_widget *w;
	struct snd_soc_dapm_context *dapm = &platform->dapm;
	int ret = 0;

	list_for_each_entry(w, &dapm->card->widgets, list) {
		if (w->platform && is_sst_dapm_widget(w) && (w->priv)) {
			struct sst_ids *ids = w->priv;

			pr_debug("widget type=%d name=%s", w->id, w->name);
			INIT_LIST_HEAD(&ids->algo_list);
			INIT_LIST_HEAD(&ids->gain_list);
			ret = sst_fill_widget_module_info(w, platform);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

int sst_dsp_init_v2_dpcm(struct snd_soc_platform *platform)
{
	int i, ret = 0;
	struct sst_data *sst = snd_soc_platform_get_drvdata(platform);

	sst->byte_stream = devm_kzalloc(platform->dev,
					SST_MAX_BIN_BYTES, GFP_KERNEL);
	if (!sst->byte_stream) {
		pr_err("%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}
	sst->widget = devm_kzalloc(platform->dev,
				   SST_NUM_WIDGETS * sizeof(*sst->widget),
				   GFP_KERNEL);
	if (!sst->widget) {
		pr_err("%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	snd_soc_dapm_new_controls(&platform->dapm, sst_dapm_widgets,
			ARRAY_SIZE(sst_dapm_widgets));
	snd_soc_dapm_add_routes(&platform->dapm, intercon,
			ARRAY_SIZE(intercon));
	snd_soc_dapm_new_widgets(&platform->dapm);

	for (i = 0; i < SST_NUM_GAINS; i++) {
		sst_gains[i].mute = SST_GAIN_MUTE_DEFAULT;
		sst_gains[i].l_gain = SST_GAIN_VOLUME_DEFAULT;
		sst_gains[i].r_gain = SST_GAIN_VOLUME_DEFAULT;
		sst_gains[i].ramp_duration = SST_GAIN_RAMP_DURATION_DEFAULT;
	}

	snd_soc_add_platform_controls(platform, sst_gain_controls,
			ARRAY_SIZE(sst_gain_controls));

	snd_soc_add_platform_controls(platform, sst_algo_controls,
			ARRAY_SIZE(sst_algo_controls));
	snd_soc_add_platform_controls(platform, sst_slot_controls,
			ARRAY_SIZE(sst_slot_controls));
	snd_soc_add_platform_controls(platform, sst_mux_controls,
			ARRAY_SIZE(sst_mux_controls));
	snd_soc_add_platform_controls(platform, sst_debug_controls,
			ARRAY_SIZE(sst_debug_controls));

	/* initialize the names of the probe points */
	for (i = 0; i < SST_NUM_PROBE_CONNECTION_PTS; i++)
		sst_probe_enum_texts[i] = sst_probes[i].name;

	snd_soc_add_platform_controls(platform, sst_probe_controls,
			ARRAY_SIZE(sst_probe_controls));

	ret = sst_map_modules_to_pipe(platform);

	return ret;
}
