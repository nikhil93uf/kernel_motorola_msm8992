/* Copyright (c) 2015, Alok Nikhil <aloknnikhil@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/lcd_notify.h>
#include <linux/cpufreq.h>

static bool isSuspended = false;

struct notifier_block display_worker;

#define X_PLUG_DEBUG 1

#define X_PLUG "xplug"

#define DRIVER_VERSION  1
#define DRIVER_SUBVER 8

typedef enum {
	DISABLED,
	IDLE,
	DOWN,
	UP,
} XPLUG_STATE;

#define TARGET_LOAD 1
#define TARGET_THERMAL 2
#define TARGET_HISTORY 3
#define TARGET_PREDICT 4

/* configurable parameters */
static unsigned int sample_rate = 250;		/* msec */
static unsigned int max_cpus = 6;
static unsigned int min_cpus = 1;
static unsigned int touch_boost_enabled = 1;

/* 1 - target_load; 2 - target_thermal; 3 - target_history; 4 - target_predict */
static unsigned int policy = 1;		
static void policy_function(void (*cpu_policy)(void))	{	cpu_policy();	}

/* target_load parameters */	
static unsigned int target_load = 40;
static unsigned int dispatch_rate = 2;
static unsigned int biased_down_up = 0; /* 0 - Offline faster; 1 - Online faster */
static void target_load_policy(void);

/* target_predict */
static void target_predict_policy(void);
static unsigned int response_index = 0;
static unsigned int min_target_load = 50;

static XPLUG_STATE xplug_state;
static struct workqueue_struct *xplug_wq;
static struct delayed_work xplug_work;

static struct workqueue_struct *xplug_boost_wq;
static struct delayed_work xplug_boost;

static struct workqueue_struct *xplug_resume_wq;
static struct delayed_work xplug_resume_work;

DEFINE_MUTEX(xplug_work_lock);

struct cpu_load_data {
	u64 prev_cpu_idle;
	u64 prev_cpu_wall;
	unsigned int avg_load_maxfreq;
	unsigned int cur_load_maxfreq;
	unsigned int samples;
	unsigned int window_size;
	cpumask_var_t related_cpus;
	unsigned int last_computed_load;
};

static DEFINE_PER_CPU(struct cpu_load_data, cpuload);

static int cpu_series[] = {0, 1, 4, 2, 3, 5};
static int curr_index = 0;

static void print_cpus_all(void);
static unsigned int get_average_load(void);

static void target_load_policy(void)	{

	unsigned int curr_load = get_average_load();
	static signed int check_count = 0;	
	int scaled_sampler = ((sample_rate * 20 * 5)/1000);

	if((curr_load) > target_load)	{
		if(biased_down_up == 1)
			check_count-=dispatch_rate;
		else
			check_count--;
	}
	else if((curr_load) < (target_load))	{
		if(biased_down_up == 0)
			check_count+=dispatch_rate;
		else
			check_count++;
	}

	pr_info("%s Current Load = %d; Check count = %d", X_PLUG, curr_load, check_count);

	if(check_count >= (scaled_sampler))		{
#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)	
		if(num_online_cpus() > 1)	
			pr_info("%s Going down\n", X_PLUG);
#endif
		xplug_state = DOWN;
		check_count = 0;
	}
	else if(check_count <= (-1 * scaled_sampler))	{
#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
		if(num_online_cpus() != nr_cpu_ids)	
			pr_info("%s Going up\n", X_PLUG);
#endif
		xplug_state = UP;
		check_count = 0;
	} else {
		xplug_state = IDLE;
	}
}

static void target_predict_policy(void)	{

	static int cpu_load[9] = {0,0,0,0,0,0,0,0,1};
	unsigned int curr_load = get_average_load();

	curr_load = curr_load/10;
		
	if((curr_load == 9) || (curr_load == 8) || (curr_load == 10))	{
		curr_load = 8;
		cpu_load[curr_load]++;

		response_index = 
			(cpu_load[curr_load - 1] > cpu_load[curr_load]) 
				? (curr_load - 1) : (curr_load);

	}
	else if (curr_load == 0)	{
		cpu_load[curr_load]++;
		response_index = (cpu_load[curr_load + 1] > cpu_load[curr_load]) 
			? (curr_load + 1) : (curr_load);
	}

	else	{
		cpu_load[curr_load]++;

		response_index = (cpu_load[curr_load - 1] > cpu_load[curr_load]) ? 
				(cpu_load[curr_load - 1] > cpu_load[curr_load + 1] ? 
				(curr_load - 1) : 
				(curr_load + 1)) : 
				(cpu_load[curr_load] > cpu_load[curr_load + 1] ? 
				(curr_load) : (curr_load + 1));
	}
	
	response_index *= 10;
	target_load = 100 - response_index;

	if(target_load < min_target_load)
		target_load = min_target_load;

#ifdef X_PLUG_DEBUG
	pr_info("%s Current load history - %d|%d|%d|%d|%d|%d|%d|%d|%d\n", X_PLUG,
				cpu_load[0], cpu_load[1],
				cpu_load[2], cpu_load[3],
				cpu_load[4], cpu_load[5],
				cpu_load[6], cpu_load[7],
				cpu_load[8]);
	pr_info("%s Current load target - %d\n", X_PLUG, target_load);
#endif
	policy_function(&target_load_policy);
}

static void update_xplug_state(void)
{
	switch(policy)	{
	case TARGET_LOAD : policy_function(&target_load_policy);
		 break;
	case TARGET_PREDICT : policy_function(&target_predict_policy);
		 break;	
	}
}

static void __cpuinit xplug_work_fn(struct work_struct *work)
{
	bool up = false;
	bool sample = false;
	unsigned int cpu = nr_cpu_ids;

	if(isSuspended)
		

	mutex_lock(&xplug_work_lock);

	update_xplug_state();

	switch (xplug_state) {
	case DISABLED:
		break;
	case IDLE:
		sample = true;
		break;
	case UP:
		cpu = cpu_series[++curr_index];
		up = true;
		sample = true;
		xplug_state = IDLE;
		break;
	case DOWN:
		cpu = cpu_series[curr_index];
		sample = true;
		xplug_state = IDLE;
		break;
	default:
		pr_err("%s: invalid state %d\n",
			__func__, xplug_state);
		break;
	}

	if (sample)
		queue_delayed_work_on(0, xplug_wq, &xplug_work,
					msecs_to_jiffies(sample_rate));

	if (cpu < NR_CPUS) {
		if(cpu >= max_cpus)	{
			cpu_down(cpu);
			curr_index--;
		}
		if (up)	{
			if(cpu < max_cpus)	{
				cpu_up(cpu);
			}
		}
		else	{
			cpu_down(cpu);
		}
	}

#ifdef X_PLUG_DEBUG
	print_cpus_all();		
#endif
	mutex_unlock(&xplug_work_lock);
}

// Utilities and Helpers

static inline void offline_cpus(void)
{
	unsigned int j;
	for(j = min_cpus ; j < NR_CPUS; j++)
	{
		if (cpu_online(j))
			cpu_down(j);
	}
	curr_index = min_cpus - 1;
#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
	pr_info("%s: %d cpus were offlined\n", X_PLUG, j);
#endif
}

static inline void cpus_online_all(void)
{
	unsigned int j;

	for(j = 1 ; j < NR_CPUS; j++)
	{
		if (cpu_is_offline(j))
			cpu_up(j);
	}
	curr_index = max_cpus - 1;
#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
	pr_info("%s: all cpus were onlined\n", X_PLUG);
#endif
	queue_delayed_work_on(0, xplug_wq, &xplug_work,
					msecs_to_jiffies(sample_rate));
}

static unsigned int get_curr_load(unsigned int cpu)
{
	int ret;
	unsigned int idle_time, wall_time;
	unsigned int cur_load;
	u64 cur_wall_time, cur_idle_time;
	struct cpu_load_data *pcpu = &per_cpu(cpuload, cpu);
	struct cpufreq_policy policy;

	ret = cpufreq_get_policy(&policy, cpu);
	if (ret)
		return -EINVAL;

	cur_idle_time = get_cpu_idle_time(cpu, &cur_wall_time, 0);

	wall_time = (unsigned int) (cur_wall_time - pcpu->prev_cpu_wall);
	pcpu->prev_cpu_wall = cur_wall_time;

	idle_time = (unsigned int) (cur_idle_time - pcpu->prev_cpu_idle);
	pcpu->prev_cpu_idle = cur_idle_time;

	if (unlikely(!wall_time || wall_time < idle_time))
		return 0;

	cur_load = 100 * (wall_time - idle_time) / wall_time;
	pcpu->last_computed_load = cur_load;
	return cur_load;
}

static unsigned int get_average_load(void)
{
	int cpu;
	unsigned int cur_load = 0;
	
	get_online_cpus();
	for_each_online_cpu(cpu) {
		cur_load += get_curr_load(cpu);
	}
	put_online_cpus();
#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
	pr_info("Total load is %d\n", cur_load);
#endif
	cur_load /= num_online_cpus();

#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
	pr_info("Per-CPU load is %d\n", cur_load);
#endif
	
	return cur_load;
}

static void xplug_suspend(void)
{
	offline_cpus();
#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
	pr_info("%s: suspend\n", X_PLUG);
#endif
}

static void __ref xplug_resume(void)
{
	cpus_online_all();
#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
	pr_info("%s: resume\n", X_PLUG);
#endif
}

static void __cpuinit xplug_resume_work_fn(struct work_struct *work)
{
	xplug_resume();
}

static void print_cpus_all(void)
{
	unsigned int cpu;

	for (cpu = 0; cpu < NR_CPUS; cpu++) {
		pr_info("%s: [%d]: %d\n", X_PLUG, cpu, cpu_is_offline(cpu)?0:1);
	}
}

// Display Event Callback Registration

static int lcd_notifier_callback(struct notifier_block *nb,
                                 unsigned long event, void *data)
{
       switch (event) {
       case LCD_EVENT_ON_START:
			isSuspended = false;
			queue_delayed_work_on(0, xplug_resume_wq, &xplug_resume_work,
		                 msecs_to_jiffies(10));
#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
			pr_info("xplug : resume called\n");
#endif
               break;
       case LCD_EVENT_ON_END:
               break;
       case LCD_EVENT_OFF_START:
               break;
       case LCD_EVENT_OFF_END:
			isSuspended = true;
			xplug_suspend();
#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
			pr_info("xplug : suspend called\n");
#endif
               break;
       default:
               break;
       }

       return 0;
}

// Input Event Callback Registration

static void __ref xplug_boost_work_fn(struct work_struct *work)
{
	int cpu;
	for(cpu = 1; cpu < NR_CPUS; cpu++) {
		if(cpu_is_offline(cpu))
			cpu_up(cpu);
	}
}

static void xplug_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{

	if ((type == EV_KEY) && (code == BTN_TOUCH) && (value == 1)
		&& touch_boost_enabled == 1)
	{
#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
			pr_info("%s : touch boost\n", X_PLUG);
#endif
		queue_delayed_work_on(0, xplug_boost_wq, &xplug_boost,
			msecs_to_jiffies(0));
	}
}

static int xplug_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void xplug_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id xplug_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler xplug_input_handler = {
	.event          = xplug_input_event,
	.connect        = xplug_input_connect,
	.disconnect     = xplug_input_disconnect,
	.name           = "xplug_handler",
	.id_table       = xplug_ids,
};

/************************** SysFS - Start **************************/

/***** Sample Rate Attribute *****/
static ssize_t sample_rate_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", sample_rate);
}

static ssize_t __ref sample_rate_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	sample_rate = val;

	return count;
}

static struct kobj_attribute sample_rate_attribute =
       __ATTR(sample_rate,
               0666,
               sample_rate_show, sample_rate_store);

/***** Policy Attribute *****/
static ssize_t policy_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", policy);
}

static ssize_t __ref policy_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	
	if(policy >= 0 && policy <= 3)
		policy = val;
	else
		pr_info("%s : invalid policy\n", X_PLUG);

	return count;
}

static struct kobj_attribute policy_attribute =
       __ATTR(policy,
               0666,
               policy_show, policy_store);

/***** Target Load Attribute *****/
static ssize_t target_load_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", target_load);
}

static ssize_t __ref target_load_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	
	if(target_load >= 0 && target_load <=100)	{
		target_load = val;
		if(policy != TARGET_LOAD)
			pr_info("%s : WARNING! The target load has \
				no effect on the current policy. Use \
				the \"Target Load\" policy for that.", X_PLUG);
	}
	else
		pr_info("%s : invalid target load\n", X_PLUG);

	return count;
}

static struct kobj_attribute target_load_attribute =
       __ATTR(target_load,
               0666,
               target_load_show, target_load_store);

/***** Dispatch Rate Attribute *****/
static ssize_t dispatch_rate_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", dispatch_rate);
}

static ssize_t __ref dispatch_rate_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	
	dispatch_rate = val;

	return count;
}

static struct kobj_attribute dispatch_rate_attribute =
       __ATTR(dispatch_rate,
               0666,
               dispatch_rate_show, dispatch_rate_store);

/***** Up/Down Bias Attribute *****/
static ssize_t biased_down_up_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", biased_down_up);
}

static ssize_t __ref biased_down_up_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	
	if(val == 0 || val == 1)
		biased_down_up = val;
	else	
		pr_info("%s : invalid bias value. Should be 0 or 1", X_PLUG);

	return count;
}

static struct kobj_attribute biased_down_up_attribute =
       __ATTR(biased_down_up,
               0666,
               biased_down_up_show, biased_down_up_store);

static struct attribute *xplug_attributes[] = {
	&sample_rate_attribute.attr,
	&policy_attribute.attr,
	&target_load_attribute.attr,
	&dispatch_rate_attribute.attr,
	&biased_down_up_attribute.attr,
	NULL,
};

static struct attribute_group xplug_attr_group =
    {
        .attrs = xplug_attributes,
    };

static struct kobject *xplug_kobj;

static int __init xplug_init(void)
{
        int ret = 0;
        int sysfs_result;
        printk(KERN_DEBUG "[%s]\n",__func__);

        xplug_kobj = kobject_create_and_add("xplug", kernel_kobj);

        if (!xplug_kobj) {
                pr_err("%s Interface create failed!\n",
                        __FUNCTION__);
                return -ENOMEM;
        }

        sysfs_result = sysfs_create_group(xplug_kobj, &xplug_attr_group);

        if (sysfs_result) {
                pr_info("%s sysfs create failed!\n", __FUNCTION__);
                kobject_put(xplug_kobj);
        }

	display_worker.notifier_call = lcd_notifier_callback;
        lcd_register_client(&display_worker);

#if defined(X_PLUG_INFO ) || defined(X_PLUG_DEBUG)
	pr_info("%s : registering input boost", X_PLUG);
#endif
	ret = input_register_handler(&xplug_input_handler);
	if (ret) {
	pr_err("%s: Failed to register input handler: %d\n",
		X_PLUG, ret);
	}

	xplug_wq = alloc_workqueue("xplug",
			WQ_HIGHPRI | WQ_UNBOUND, 1);

	xplug_resume_wq = alloc_workqueue("xplug_resume",
			WQ_HIGHPRI | WQ_UNBOUND, 1);

	xplug_boost_wq = alloc_workqueue("xplug_boost",
			WQ_HIGHPRI | WQ_UNBOUND, 1);

	INIT_DELAYED_WORK(&xplug_work, xplug_work_fn);
	INIT_DELAYED_WORK(&xplug_resume_work, xplug_resume_work_fn);
	INIT_DELAYED_WORK(&xplug_boost, xplug_boost_work_fn);
	queue_delayed_work_on(0, xplug_wq, &xplug_work, msecs_to_jiffies(10));
        pr_info("%s: init\n", X_PLUG);

        return ret;
}

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Alok Nikhil <aloknnikhil@gmail.com");
MODULE_DESCRIPTION("Hotplug driver for the MSM8992 SOC");
late_initcall(xplug_init);

