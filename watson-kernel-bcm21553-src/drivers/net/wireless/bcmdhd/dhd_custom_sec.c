#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>

#include <proto/ethernet.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_dbg.h>

#include <linux/fcntl.h>
#include <linux/fs.h>

#ifdef CUSTOMER_HW_SAMSUNG
int
dhd_read_macaddr(struct ether_addr *mac)
{
	struct file *fp		= NULL;
	struct file *fpnv	= NULL;
	char macbuffer[18]	= {0};
	mm_segment_t oldfs	= {0};
	char randommac[3]	= {0};
	char buf[18]		= {0};
	char* filepath		= "/data/.mac.info";
	char* nvfilepath	= "/data/misc/wifi/.nvmac.info";
	int ret = 0;

	//MAC address copied from nv
	fpnv = filp_open(nvfilepath, O_RDONLY, 0);
	if (IS_ERR(fpnv)) {
start_readmac:
		fpnv = NULL;
		fp = filp_open(filepath, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			/* File Doesn't Exist. Create and write mac addr.*/
			fp = filp_open(filepath, O_RDWR | O_CREAT, 0666);
			if(IS_ERR(fp)) {
				DHD_ERROR(("[WIFI] %s: File open error\n", filepath));
				return -1;
			}

			oldfs = get_fs();
			set_fs(get_ds());

			/* Generating the Random Bytes for 3 last octects of the MAC address */
			get_random_bytes(randommac, 3);

			sprintf(macbuffer,"%02X:%02X:%02X:%02X:%02X:%02X\n",
				0x12,0x34,0x56,randommac[0],randommac[1],randommac[2]);
			DHD_ERROR(("[WIFI] The Random Generated MAC ID : %s\n", macbuffer));

			if(fp->f_mode & FMODE_WRITE) {
				ret = fp->f_op->write(fp, (const char *)macbuffer, sizeof(macbuffer), &fp->f_pos);
				if(ret < 0)
					DHD_ERROR(("[WIFI] Mac address [%s] Failed to write into File: %s\n", macbuffer, filepath));
				else
					DHD_INFO(("[WIFI] Mac address [%s] written into File: %s\n", macbuffer, filepath));
			}
			set_fs(oldfs);
		}
		/* Reading the MAC Address from .mac.info file( the existed file or just created file)*/
		//rtn_value=kernel_read(fp, fp->f_pos, buf, 18);
		ret = kernel_read(fp, 0, buf, 18);
	}
	else {
		/* Reading the MAC Address from .nvmac.info file( the existed file or just created file)*/
		ret = kernel_read(fpnv, 0, buf, 18);
		buf[17] ='\0'; // to prevent abnormal string display when mac address is displayed on the screen.
		if(strncmp(buf, "00:00:00:00:00:00" , 17) == 0) {
			filp_close(fpnv, NULL);
			goto start_readmac;
		}

		fp = filp_open(filepath, O_RDWR | O_CREAT, 0666); // File is always created.
		if(IS_ERR(fp)) {
			DHD_ERROR(("[WIFI] %s: File open error\n", filepath));
			if (fpnv)
				filp_close(fpnv, NULL);
			return -1;
		}
		else {
			oldfs = get_fs();
			set_fs(get_ds());

			if(fp->f_mode & FMODE_WRITE) {
				ret = fp->f_op->write(fp, (const char *)buf, sizeof(buf), &fp->f_pos);
				if(ret < 0)
					DHD_ERROR(("[WIFI] Mac address [%s] Failed to write into File: %s\n", buf, filepath));
				else
					DHD_INFO(("[WIFI] Mac address [%s] written into File: %s\n", buf, filepath));
			}
			set_fs(oldfs);

			ret = kernel_read(fp, 0, buf, 18);
		}

	}

	if(ret)
		sscanf(buf,"%02X:%02X:%02X:%02X:%02X:%02X",
				&mac->octet[0], &mac->octet[1], &mac->octet[2],
				&mac->octet[3], &mac->octet[4], &mac->octet[5]);
	else
		DHD_ERROR(("dhd_bus_start: Reading from the '%s' returns 0 bytes\n", filepath));

	if (fp)
		filp_close(fp, NULL);
	if (fpnv)
		filp_close(fpnv, NULL);

	return 0;
}
#endif
