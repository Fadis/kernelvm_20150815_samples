#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/seq_kernel.h>
#include <sound/seq_device.h>
#include <sound/rawmidi.h>
#include <sound/info.h>
#include <sound/control.h>
#include <sound/minors.h>
#include <sound/seq_midi_event.h>
#include <sound/seq_midi_emul.h>
#include <linux/workqueue.h>

#define SNDRV_PRINTKMIDI_SUBSCRIBE  (1<<0)
#define SNDRV_PRINTKMIDI_USE  (1<<1)

struct snd_seq_ym2413_dev {
  struct work_struct work;
  struct workqueue_struct *work_queue;
  struct snd_card *card;
  int seq_client;
  struct snd_midi_channel_set * chset;
  int device;
  int client;   /* created/attached client */
  int port;   /* created/attached port */
  unsigned int flags;  /* SNDRV_PRINTKMIDI_* */
  raw_spinlock_t bus_lock;
  struct i2c_client *bus;
  int current_program[16];
  unsigned char buffer[128];
  unsigned char *buffer_head;
};

struct snd_ym2413 {
  struct snd_card *card;
  struct snd_seq_device *seq;
	 struct device dev;
  struct snd_seq_ym2413_dev *rdev;
};

int __i2c_master_send(const struct i2c_client *client, const char *buf, int count) {
  int ret;
  struct i2c_adapter *adap = client->adapter;
  struct i2c_msg msg;
  msg.addr = client->addr;
  msg.flags = client->flags & I2C_M_TEN;
  msg.len = count;
  msg.buf = (char *)buf;

  ret = __i2c_transfer(adap, &msg, 1);

  /*
   * If everything went ok (i.e. 1 msg transmitted), return #bytes
   * transmitted, else error code.
   */
  return (ret == 1) ? count : ret;
}

static void snd_ym2413_worker( struct work_struct *work ) {
  unsigned long flags;
  struct snd_seq_ym2413_dev *rdev;
  unsigned char buffer[128];
  unsigned int buffer_size = 0;
  rdev = (struct snd_seq_ym2413_dev*)work;
  raw_spin_lock_irqsave(&rdev->bus_lock, flags);
  buffer_size = rdev->buffer_head - rdev->buffer;
  memcpy( buffer, rdev->buffer, buffer_size );
  rdev->buffer_head = rdev->buffer;
  raw_spin_unlock_irqrestore(&rdev->bus_lock, flags);
  i2c_master_send( rdev->bus, buffer, buffer_size );
}

void snd_ym2413_note_on(void *p, int note, int vel, struct snd_midi_channel *chan) {
  unsigned long flags;
  bool send_program_change = false;
  struct snd_seq_ym2413_dev *rdev;
  rdev = p;
  if( chan->midi_program != rdev->current_program[ chan->number ] ) {
    rdev->current_program[ chan->number ] = chan->midi_program;
    send_program_change = true;
  }
  if( send_program_change ) {
    if( rdev->buffer_head < rdev->buffer + 123 ) {
      raw_spin_lock_irqsave(&rdev->bus_lock, flags);
      *rdev->buffer_head = 0x50 | chan->number;
      ++rdev->buffer_head;
      *rdev->buffer_head = chan->midi_program;
      ++rdev->buffer_head;
      *rdev->buffer_head = 0x10 | chan->number;
      ++rdev->buffer_head;
      *rdev->buffer_head = note;
      ++rdev->buffer_head;
      *rdev->buffer_head = vel * chan->gm_volume * chan->gm_expression / 128 / 128;
      ++rdev->buffer_head;
      raw_spin_unlock_irqrestore(&rdev->bus_lock, flags);
    }
  }
  else {
    if( rdev->buffer_head < rdev->buffer + 125 ) {
      raw_spin_lock_irqsave(&rdev->bus_lock, flags);
      *rdev->buffer_head = 0x10 | chan->number;
      ++rdev->buffer_head;
      *rdev->buffer_head = note;
      ++rdev->buffer_head;
      *rdev->buffer_head = vel * chan->gm_volume * chan->gm_expression / 128 / 128;
      ++rdev->buffer_head;
      raw_spin_unlock_irqrestore(&rdev->bus_lock, flags);
    }
  }
  queue_work( rdev->work_queue, (struct work_struct *)rdev );
}

void snd_ym2413_note_off(void *p, int note, int vel, struct snd_midi_channel *chan) {
  unsigned long flags;
  struct snd_seq_ym2413_dev *rdev;
  rdev = p;
  if( rdev->buffer_head < rdev->buffer + 126 ) {
    raw_spin_lock_irqsave(&rdev->bus_lock, flags);
    *rdev->buffer_head = 0x20 | chan->number;
    ++rdev->buffer_head;
    *rdev->buffer_head = note;
    ++rdev->buffer_head;
    raw_spin_unlock_irqrestore(&rdev->bus_lock, flags);
  }
  queue_work( rdev->work_queue, (struct work_struct *)rdev );
}

void snd_ym2413_key_press(void *p, int note, int vel, struct snd_midi_channel *chan) {
}

void snd_ym2413_terminate_note(void *p, int note, struct snd_midi_channel *chan) {
  unsigned long flags;
  struct snd_seq_ym2413_dev *rdev;
  rdev = p;
  if( rdev->buffer_head < rdev->buffer + 126 ) {
    raw_spin_lock_irqsave(&rdev->bus_lock, flags);
    *rdev->buffer_head = 0x20 | chan->number;
    ++rdev->buffer_head;
    *rdev->buffer_head = note;
    ++rdev->buffer_head;
    raw_spin_unlock_irqrestore(&rdev->bus_lock, flags);
  }
  queue_work( rdev->work_queue, (struct work_struct *)rdev );
}

void snd_ym2413_control(void *p, int type, struct snd_midi_channel *chan) {
  unsigned long flags;
  struct snd_seq_ym2413_dev *rdev;
  rdev = p;
  if( type == SNDRV_SEQ_EVENT_PITCHBEND ) {
    //unsigned int pitch = chan->midi_pitchbend + 8192;
    raw_spin_lock_irqsave(&rdev->bus_lock, flags);
//    i2c_smbus_write_byte( rdev->bus, 0x40 | chan->number );
//    i2c_smbus_write_byte( rdev->bus, pitch & 0x7F );
//    i2c_smbus_write_byte( rdev->bus, pitch >> 7 );
    raw_spin_unlock_irqrestore(&rdev->bus_lock,flags);
  }
}

void snd_ym2413_nrpn(void *p, struct snd_midi_channel *chan, struct snd_midi_channel_set *chset) {
}

void snd_ym2413_sysex(void *p, unsigned char *buf, int len, int parsed, struct snd_midi_channel_set *chset) {
}

struct snd_midi_op ym2413_ops = {
  .note_on =  snd_ym2413_note_on,
  .note_off =  snd_ym2413_note_off,
  .key_press =  snd_ym2413_key_press,
  .note_terminate = snd_ym2413_terminate_note,
  .control =  snd_ym2413_control,
  .nrpn =   snd_ym2413_nrpn,
  .sysex =  snd_ym2413_sysex,
};

/*
 * event input callback - just redirect events to subscribers
 */
static int ym2413_input(struct snd_seq_event *ev, int direct, void *private_data,
	    int atomic, int hop) {
	 struct snd_seq_ym2413_dev *rdev;
  rdev = private_data;
  if (!(rdev->flags & SNDRV_PRINTKMIDI_USE))
    return 0; /* ignored */
  snd_midi_process_event( &ym2413_ops, ev, rdev->chset );
  return 0;
}

static int ym2413_subscribe(void *private_data, struct snd_seq_port_subscribe *info) {
  struct snd_seq_ym2413_dev *rdev;
  rdev = private_data;
  if (!try_module_get(rdev->card->module))
    return -EFAULT;
  rdev->flags |= SNDRV_PRINTKMIDI_SUBSCRIBE;
  return 0;
}

static int ym2413_unsubscribe(void *private_data, struct snd_seq_port_subscribe *info) {
  struct snd_seq_ym2413_dev *rdev;
  rdev = private_data;
  rdev->flags &= ~SNDRV_PRINTKMIDI_SUBSCRIBE;
  module_put(rdev->card->module);
  return 0;
}

static int ym2413_use(void *private_data, struct snd_seq_port_subscribe *info) {
  struct snd_seq_ym2413_dev *rdev;
  rdev = private_data;
  if (!try_module_get(rdev->card->module))
    return -EFAULT;
  rdev->flags |= SNDRV_PRINTKMIDI_USE;
  return 0;
}

static int ym2413_unuse(void *private_data, struct snd_seq_port_subscribe *info) {
  struct snd_seq_ym2413_dev *rdev;
  rdev = private_data;
  rdev->flags &= ~SNDRV_PRINTKMIDI_USE;
  module_put(rdev->card->module);
  return 0;
}


/*
 * create a port
 */
static struct snd_seq_ym2413_dev *create_port( int client, int type ) {
	 struct snd_seq_port_info pinfo;
	 struct snd_seq_port_callback pcb;
	 struct snd_seq_ym2413_dev *rdev;
	 if ((rdev = kzalloc(sizeof(*rdev), GFP_KERNEL)) == NULL) return NULL;
		rdev->client = client;
		memset(&pinfo, 0, sizeof(pinfo));
		pinfo.addr.client = client;
  strcpy( pinfo.name, "ym2413" );
		pinfo.capability = SNDRV_SEQ_PORT_CAP_READ | SNDRV_SEQ_PORT_CAP_SUBS_READ;
		pinfo.capability |= SNDRV_SEQ_PORT_CAP_WRITE | SNDRV_SEQ_PORT_CAP_SUBS_WRITE;
  pinfo.capability |= SNDRV_SEQ_PORT_CAP_DUPLEX;
  pinfo.type = SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC
     | SNDRV_SEQ_PORT_TYPE_SOFTWARE
     | SNDRV_SEQ_PORT_TYPE_PORT;
  pinfo.midi_channels = 16;
		memset(&pcb, 0, sizeof(pcb));
		pcb.owner = THIS_MODULE;
  pcb.subscribe = ym2413_subscribe;
  pcb.unsubscribe = ym2413_unsubscribe;
  pcb.use = ym2413_use;
  pcb.unuse = ym2413_unuse;
  pcb.event_input = ym2413_input;
		pcb.private_data = rdev;
		pinfo.kernel = &pcb;
		if ( snd_seq_kernel_client_ctl(client, SNDRV_SEQ_IOCTL_CREATE_PORT, &pinfo) < 0 ) {
	 	 kfree(rdev);
		  return NULL;
	 }
	 rdev->port = pinfo.addr.port;
  memset( rdev->current_program, 0, sizeof(int)*16);
  raw_spin_lock_init(&rdev->bus_lock);
  return rdev;
}

static int snd_ym2413_probe(
  struct i2c_client *bus,
  const struct i2c_device_id *id_
) {
  struct snd_card *card;
  struct snd_ym2413 *ym2413;
  struct snd_seq_ym2413_dev *rdev;
  int err;
  int client;
  int status;
  char buffer[ 1 ];
  bus->dev.platform_data = NULL;
  status = i2c_master_recv( bus, buffer, 1 );
  if( status < 0 ) {
    dev_err( &bus->dev, "Unable to read from device: %d\n", status );
    return err;
  }
  card = NULL;
  err = snd_card_new(
    &bus->dev, -1, NULL, THIS_MODULE, sizeof(struct snd_ym2413 ), &card
  );
  if( err < 0 ) {
    dev_err( &bus->dev, "Unable to create snd_card: %d\n", err );
    return err;
  }
  ym2413 = card->private_data;
  ym2413->card = card;

 	/* create client */
	 client = snd_seq_create_kernel_client( card, 0, "ym2413" );
	 if ( client < 0 ) {
    snd_card_free( card );
		  return client;
  }
	 rdev = create_port( client, 0 );
	 if ( rdev == NULL ) {
	  	snd_seq_delete_kernel_client( client );
    snd_card_free( card );
		  return -ENOMEM;
	 }
  ym2413->rdev = rdev;
  rdev->buffer_head = rdev->buffer;
  rdev->work_queue = NULL;
  rdev->card = card;
  rdev->chset = snd_midi_channel_alloc_set( 16 );
  if ( rdev->chset == NULL ) {
    kfree( rdev );
	  	snd_seq_delete_kernel_client( client );
    snd_card_free( card );
    err = -ENOMEM;
  }
  rdev->bus = bus;
  rdev->client = client;
  rdev->chset->client = client;
  rdev->chset->private_data = rdev;
  printk("debug2 %lx\n", (unsigned long)rdev->bus );
  rdev->work_queue = create_workqueue("ym2413_queue");
  INIT_WORK( (struct work_struct *)rdev, snd_ym2413_worker );

  strcpy(card->driver, "ym2413");
  strcpy(card->shortname, "ym2413");
  sprintf(card->longname, "ym2413 Card %i", card->number);
  err = snd_card_register(card);
  if (!err) {
    bus->dev.platform_data = card;
    return 0;
  }
  kfree( rdev );
	 snd_seq_delete_kernel_client( client );
  snd_card_free(card);
  return err;
}

static int snd_ym2413_remove( struct i2c_client *bus ) {
  struct snd_card *card;
  struct snd_ym2413 *ym2413;
  struct snd_seq_ym2413_dev *rdev;
  card = bus->dev.platform_data;
  if( card ) {
    ym2413 = card->private_data;
    if( ym2413 ) {
      rdev = ym2413->rdev;
      if( rdev ) {
        if( rdev->work_queue ) {
          flush_workqueue( rdev->work_queue );
          destroy_workqueue( rdev->work_queue );
        }
        snd_midi_channel_free_set( rdev->chset );
        snd_seq_delete_kernel_client( rdev->client );
        kfree( rdev );
      }
    }
    snd_card_free( card );
  }
  return 0;
}

static struct i2c_device_id snd_ym2413_idtable[] = {
  { "ym2413", 0 },
  { }
};


static struct i2c_driver snd_ym2413_driver = {
  .driver = {
    .name = "snd_ym2413",
  },
  .probe    = snd_ym2413_probe,
  .remove   = snd_ym2413_remove,
  .id_table = snd_ym2413_idtable,
};

module_i2c_driver( snd_ym2413_driver );

MODULE_AUTHOR("Naomasa Matsubayashi <>");
MODULE_DESCRIPTION("YM2413 I2C Client Driver");
MODULE_LICENSE("GPL");

