/*
 * CPU idle support for KVM Guests
 *
 * This work is licensed under the terms of the GNU GPL, version 2.
 *
 * This cpu idle driver implements three idle states -
 * #1 Poll in guest mode. Wait for host to interrupt us.
 * #2 Exit to host and return immediately. Gives host chance to reschedule.
 * #3 HALT VCPU. Can make host CPU idle.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/cpuidle.h>
#include <linux/io.h>
#include <linux/export.h>
#include <asm/kvm_para.h>
#include <linux/module.h>

#define KVM_MAX_STATES	2

static bool disable = false;
module_param(disable, bool, S_IRUGO | S_IWUSR);
static int poll_cnt = 0;
module_param(poll_cnt, int, S_IRUGO | S_IWUSR);
static int exit_cnt = 0;
module_param(exit_cnt, int, S_IRUGO | S_IWUSR);
static int halt_cnt = 0;
module_param(halt_cnt, int, S_IRUGO | S_IWUSR);
static int poll_lat = 0;
module_param(poll_lat, int, S_IRUGO | S_IWUSR);
static int exit_lat = 10;
module_param(exit_lat, int, S_IRUGO | S_IWUSR);
static int halt_lat = 20;
module_param(halt_lat, int, S_IRUGO | S_IWUSR);
static int poll_res = 0;
module_param(poll_res, int, S_IRUGO | S_IWUSR);
static int exit_res = 0;
module_param(exit_res, int, S_IRUGO | S_IWUSR);
static int halt_res = 0;
module_param(halt_res, int, S_IRUGO | S_IWUSR);

static int kvm_enter_poll(struct cpuidle_device *dev,
			  struct cpuidle_driver *drv,
			  int index)
{
	++poll_cnt;
	return index;
}

static int kvm_enter_outb(struct cpuidle_device *dev,
			  struct cpuidle_driver *drv,
			  int index)
{
	++exit_cnt;
	outb(0x0, 0xff04U);
	return index;
}

static int kvm_enter_idle(struct cpuidle_device *dev,
			  struct cpuidle_driver *drv,
			  int index)
{
	++halt_cnt;
	safe_halt();
	return index;
}

static struct cpuidle_driver kvm_cpuidle_driver = {
	.name			= "kvm_idle",
	.owner			= THIS_MODULE,
	.states[0]		= {
		.enter			= kvm_enter_poll,
		.exit_latency		= 0,
		.target_residency	= 0,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "poll_idle",
		.desc			= "Polling idle mode",
	},
	.states[1]		= {
		.enter			= kvm_enter_outb,
		.exit_latency		= 10,
		.target_residency	= 0,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "halt_pio",
		.desc			= "Halt idle mode",
	},
	.states[2]		= {
		.enter			= kvm_enter_idle,
		.exit_latency		= 100,
		.target_residency	= 0,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "halt_idle",
		.desc			= "Halt idle mode",
	},
	.state_count = 3,
};

/* Initialize CPU idle by registering the idle states */
static int kvm_idle_init_module(void)
{
	int r;
	if (disable)
		return 0;

	kvm_cpuidle_driver.states[0].exit_latency = poll_lat;
	kvm_cpuidle_driver.states[1].exit_latency = exit_lat;
	kvm_cpuidle_driver.states[2].exit_latency = halt_lat;

	kvm_cpuidle_driver.states[0].target_residency = poll_res;
	kvm_cpuidle_driver.states[1].target_residency = exit_res;
	kvm_cpuidle_driver.states[2].target_residency = halt_res;


	if (!kvm_para_available())
		return -ENOTTY;
	r = cpuidle_register(&kvm_cpuidle_driver, NULL);
	return r;
}

static void kvm_idle_cleanup_module(void)
{
	if (disable)
		return;
	cpuidle_unregister_driver(&kvm_cpuidle_driver);
}

module_init(kvm_idle_init_module);
module_exit(kvm_idle_cleanup_module);

