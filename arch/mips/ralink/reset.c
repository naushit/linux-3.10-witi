/**************************************************************************
 *
 *  BRIEF MODULE DESCRIPTION
 *     reboot/reset setting for Ralink RT2880 solution
 *
 *  Copyright 2007 Ralink Inc. (bruce_chang@ralinktech.com.tw)
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 **************************************************************************
 * May 2007 Bruce Chang
 *
 * Initial Release
 *
 *
 *
 **************************************************************************
 */



#include <asm/reboot.h>
#include <asm/mach-ralink/generic.h>
#include <linux/pm.h>
#include <linux/delay.h>

#if defined (CONFIG_OF)
#include <linux/io.h>
#include <linux/of.h>
#include <linux/reset-controller.h>
#include <asm/mach-ralink/ralink_regs.h>
#endif

static void mips_machine_restart(char *command);
static void mips_machine_halt(void);
static void mips_machine_power_off(void);

static void mips_machine_restart(char *command)
{
	*(volatile unsigned int*)(SOFTRES_REG) = GORESET;
	*(volatile unsigned int*)(SOFTRES_REG) = 0;
}

static void mips_machine_halt(void)
{
	*(volatile unsigned int*)(SOFTRES_REG) = (0x1)<<26; // PCIERST
	mdelay(10);
	*(volatile unsigned int*)(SOFTRES_REG) = GORESET;
	*(volatile unsigned int*)(SOFTRES_REG) = 0;
}

static void mips_machine_power_off(void)
{
	*(volatile unsigned int*)(POWER_DIR_REG) = POWER_DIR_OUTPUT;
	*(volatile unsigned int*)(POWER_POL_REG) = 0;
	*(volatile unsigned int*)(POWEROFF_REG) = POWEROFF;
}


void mips_reboot_setup(void)
{
	_machine_restart = mips_machine_restart;
	_machine_halt = mips_machine_halt;
	//_machine_power_off = mips_machine_power_off;
	pm_power_off = mips_machine_power_off;
}
#if defined (CONFIG_OF)
/* Reset Control */
#define SYSC_REG_RESET_CTRL     0x034
#define RSTCTL_RESET_SYSTEM     BIT(0)

static int ralink_assert_device(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	u32 val;

	if (id < 8)
		return -1;

	val = rt_sysc_r32(SYSC_REG_RESET_CTRL);
	val |= BIT(id);
	rt_sysc_w32(val, SYSC_REG_RESET_CTRL);

	return 0;
}

static int ralink_deassert_device(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	u32 val;

	if (id < 8)
		return -1;

	val = rt_sysc_r32(SYSC_REG_RESET_CTRL);
	val &= ~BIT(id);
	rt_sysc_w32(val, SYSC_REG_RESET_CTRL);

	return 0;
}

static int ralink_reset_device(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	ralink_assert_device(rcdev, id);
	return ralink_deassert_device(rcdev, id);
}


static struct reset_control_ops reset_ops = {
	.reset = ralink_reset_device,
	.assert = ralink_assert_device,
	.deassert = ralink_deassert_device,
};

static struct reset_controller_dev reset_dev = {
	.ops			= &reset_ops,
	.owner			= THIS_MODULE,
	.nr_resets		= 32,
	.of_reset_n_cells	= 1,
};

void ralink_rst_init(void)
{
	reset_dev.of_node = of_find_compatible_node(NULL, NULL,
						"ralink,rt2880-reset");
	if (!reset_dev.of_node)
		pr_err("Failed to find reset controller node");
	else
		reset_controller_register(&reset_dev);
}
#endif