#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xd31aec96, "module_layout" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x85bd1608, "__request_region" },
	{ 0x8ada0990, "netdev_info" },
	{ 0x52d1fb52, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0x1fdc7df2, "_mcount" },
	{ 0x974e7352, "register_candev" },
	{ 0xb68311f4, "pci_free_irq_vectors" },
	{ 0xac32588b, "pci_write_config_word" },
	{ 0x349cba85, "strchr" },
	{ 0xea9e797f, "single_open" },
	{ 0x77358855, "iomem_resource" },
	{ 0x98cf60b3, "strlen" },
	{ 0xcf0b4663, "alloc_can_err_skb" },
	{ 0x43d253ef, "dma_set_mask" },
	{ 0x9972f3c5, "single_release" },
	{ 0xef7d4e1d, "usb_reset_endpoint" },
	{ 0x340c0a0c, "pci_disable_device" },
	{ 0xd8e63fda, "i2c_transfer" },
	{ 0xd27de267, "netif_carrier_on" },
	{ 0x12a4e128, "__arch_copy_from_user" },
	{ 0x20000329, "simple_strtoul" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0x3ae5977f, "seq_printf" },
	{ 0x56470118, "__warn_printk" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x759ea8c3, "usb_kill_urb" },
	{ 0x536ce8c, "remove_proc_entry" },
	{ 0xb0a8579b, "device_destroy" },
	{ 0x13787c35, "__register_chrdev" },
	{ 0xe9e13e55, "driver_for_each_device" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0xd78c9a40, "pci_release_regions" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0x409bcb62, "mutex_unlock" },
	{ 0x50a88ad6, "dma_free_attrs" },
	{ 0x73c6ae3a, "device_create_with_groups" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x33b21099, "seq_read" },
	{ 0x6c54880, "dma_set_coherent_mask" },
	{ 0x15ba50a6, "jiffies" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x59eb6b88, "can_bus_off" },
	{ 0x25f1111d, "netif_rx" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xf7d8097b, "dma_get_required_mask" },
	{ 0x44e8dc2e, "param_ops_charp" },
	{ 0xd11b65e5, "pci_set_master" },
	{ 0xd8ff7f62, "pci_alloc_irq_vectors_affinity" },
	{ 0xd8b904ac, "_dev_warn" },
	{ 0xdcb764ad, "memset" },
	{ 0xdbdf6c92, "ioport_resource" },
	{ 0x9c5e6bea, "close_candev" },
	{ 0x1e1e140e, "ns_to_timespec64" },
	{ 0xba7e6623, "netif_tx_wake_queue" },
	{ 0x4b0a3f52, "gic_nonsecure_priorities" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x37befc70, "jiffies_to_msecs" },
	{ 0xbf2d5410, "usb_deregister" },
	{ 0x977f511b, "__mutex_init" },
	{ 0xc5850110, "printk" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0xfef216eb, "_raw_spin_trylock" },
	{ 0xff8a30b8, "sysfs_remove_file_from_group" },
	{ 0x449ad0a7, "memcmp" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0xace17747, "class_unregister" },
	{ 0x1edb69d6, "ktime_get_raw_ts64" },
	{ 0x6fd70b1c, "usb_set_interface" },
	{ 0xb8a5b44b, "free_netdev" },
	{ 0x9166fada, "strncpy" },
	{ 0xec3f4348, "usb_control_msg" },
	{ 0x5a921311, "strncmp" },
	{ 0x1cd2b4ff, "pci_read_config_word" },
	{ 0x3f3887d7, "dma_alloc_attrs" },
	{ 0x2ab7989d, "mutex_lock" },
	{ 0x1e6d26a8, "strstr" },
	{ 0x76d77b86, "alloc_candev_mqs" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x6b4b2933, "__ioremap" },
	{ 0x70991ba5, "init_net" },
	{ 0x27b8f016, "__class_register" },
	{ 0x59982873, "_dev_err" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x167c5967, "print_hex_dump" },
	{ 0x43fa3ee4, "can_change_mtu" },
	{ 0x30ea02a1, "i2c_del_adapter" },
	{ 0xae0dc4e9, "_dev_info" },
	{ 0x1ca7fb0d, "usb_submit_urb" },
	{ 0xb6a1725c, "unregister_candev" },
	{ 0xf017e439, "alloc_can_skb" },
	{ 0x12a38747, "usleep_range" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0x86332725, "__stack_chk_fail" },
	{ 0xdf0fe86f, "usb_reset_device" },
	{ 0xc6d17a7c, "usb_bulk_msg" },
	{ 0x1000e51, "schedule" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb8dce12, "kfree_skb" },
	{ 0x18a844c9, "usb_clear_halt" },
	{ 0x49f2fa43, "cpu_hwcaps" },
	{ 0xf424b0fe, "cpu_hwcap_keys" },
	{ 0xdd56d605, "netdev_err" },
	{ 0x1035c7c2, "__release_region" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x80aadbdf, "pci_unregister_driver" },
	{ 0xcc5005fe, "msleep_interruptible" },
	{ 0xa9842b7e, "__dev_get_by_name" },
	{ 0x537b7dc7, "open_candev" },
	{ 0x9ac23f86, "kmem_cache_alloc_trace" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x425b80d9, "param_ops_byte" },
	{ 0x5f4fae80, "pci_irq_vector" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xf6ebc03b, "net_ratelimit" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x28d44d0, "seq_lseek" },
	{ 0x37a0cba, "kfree" },
	{ 0x4829a47e, "memcpy" },
	{ 0xbcc9f7ff, "pci_request_regions" },
	{ 0x391baf7f, "param_array_ops" },
	{ 0xaf56600a, "arm64_use_ng_mappings" },
	{ 0xedc03953, "iounmap" },
	{ 0xa5eadf27, "__pci_register_driver" },
	{ 0x571fd711, "usb_register_driver" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x4df073e3, "alloc_canfd_skb" },
	{ 0xa2a49072, "sysfs_add_file_to_group" },
	{ 0x1705d111, "i2c_bit_add_bus" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x5a9f1d63, "memmove" },
	{ 0xbadb094d, "pci_iomap" },
	{ 0xef66446f, "consume_skb" },
	{ 0x2ad1d0f7, "param_ops_ushort" },
	{ 0x683a2d20, "proc_create" },
	{ 0x923b67ce, "usb_get_current_frame_number" },
	{ 0x5e515be6, "ktime_get_ts64" },
	{ 0xa3816e0d, "pci_enable_device" },
	{ 0x948732d9, "param_ops_ulong" },
	{ 0x82ea9962, "param_ops_uint" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0xcb855872, "usb_free_urb" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0x655fc02e, "usb_alloc_urb" },
	{ 0xc1514a3b, "free_irq" },
};

MODULE_INFO(depends, "can-dev");

MODULE_ALIAS("pci:v0000001Cd00000001sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000003sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000004sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000005sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000006sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000007sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000008sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000009sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000002sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000000Asv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000010sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000013sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000014sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000016sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000017sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000018sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000019sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000001Asv*sd*bc*sc*i*");
MODULE_ALIAS("usb:v0C72p000Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p000Dd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0012d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0011d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0013d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0014d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "7192938F72EDF175772E883");
