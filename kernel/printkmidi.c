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
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#define SNDRV_PRINTKMIDI_SUBSCRIBE  (1<<0)
#define SNDRV_PRINTKMIDI_USE  (1<<1)

struct snd_seq_printk_dev {
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
  int current_program[16];
  unsigned char buffer[128];
  unsigned char *buffer_head;
};

struct snd_printk {
  struct snd_card *card;
  struct snd_seq_device *seq;
	 struct device dev;
  struct snd_seq_printk_dev *rdev;
};

static void snd_printk_worker( struct work_struct *work ) {
  unsigned long flags;
  struct snd_seq_printk_dev *rdev;
  unsigned char *cur;
  rdev = (struct snd_seq_printk_dev*)work;
  raw_spin_lock_irqsave(&rdev->bus_lock, flags);
  for( cur = rdev->buffer; cur != rdev->buffer_head; ++cur ) {
    printk( "%x ", (int)*cur );
  }
  rdev->buffer_head = rdev->buffer;
  raw_spin_unlock_irqrestore(&rdev->bus_lock, flags);
  printk( "\n" );
}

void snd_printk_note_on(void *p, int note, int vel, struct snd_midi_channel *chan) {
  unsigned long flags;
  bool send_program_change = false;
  struct snd_seq_printk_dev *rdev;
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

void snd_printk_note_off(void *p, int note, int vel, struct snd_midi_channel *chan) {
  unsigned long flags;
  struct snd_seq_printk_dev *rdev;
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

void snd_printk_key_press(void *p, int note, int vel, struct snd_midi_channel *chan) {
}

void snd_printk_terminate_note(void *p, int note, struct snd_midi_channel *chan) {
  unsigned long flags;
  struct snd_seq_printk_dev *rdev;
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

void snd_printk_control(void *p, int type, struct snd_midi_channel *chan) {
  unsigned long flags;
  struct snd_seq_printk_dev *rdev;
  rdev = p;
  if( type == SNDRV_SEQ_EVENT_PITCHBEND ) {
    unsigned int pitch = chan->midi_pitchbend + 8192;
    raw_spin_lock_irqsave(&rdev->bus_lock, flags);
    printk( "Pitch: %d %d\n", chan->number, pitch );
    raw_spin_unlock_irqrestore(&rdev->bus_lock, flags);
  }
}

void snd_printk_nrpn(void *p, struct snd_midi_channel *chan, struct snd_midi_channel_set *chset) {
}

void snd_printk_sysex(void *p, unsigned char *buf, int len, int parsed, struct snd_midi_channel_set *chset) {
}

struct snd_midi_op printk_ops = {
  .note_on =  snd_printk_note_on,
  .note_off =  snd_printk_note_off,
  .key_press =  snd_printk_key_press,
  .note_terminate = snd_printk_terminate_note,
  .control =  snd_printk_control,
  .nrpn =   snd_printk_nrpn,
  .sysex =  snd_printk_sysex,
};

/*
 * event input callback - just redirect events to subscribers
 */
static int printk_input(struct snd_seq_event *ev, int direct, void *private_data,
	    int atomic, int hop) {
	 struct snd_seq_printk_dev *rdev;
  rdev = private_data;
  if (!(rdev->flags & SNDRV_PRINTKMIDI_USE))
    return 0; /* ignored */
  snd_midi_process_event( &printk_ops, ev, rdev->chset );
  return 0;
}

/*
 * free_private callback
 */
static void printk_free(void *private_data) {}

static int printk_subscribe(void *private_data, struct snd_seq_port_subscribe *info) {
  struct snd_seq_printk_dev *rdev;
  rdev = private_data;
  if (!try_module_get(rdev->card->module))
    return -EFAULT;
  rdev->flags |= SNDRV_PRINTKMIDI_SUBSCRIBE;
  return 0;
}

static int printk_unsubscribe(void *private_data, struct snd_seq_port_subscribe *info) {
  struct snd_seq_printk_dev *rdev;
  rdev = private_data;
  rdev->flags &= ~SNDRV_PRINTKMIDI_SUBSCRIBE;
  module_put(rdev->card->module);
  return 0;
}

static int printk_use(void *private_data, struct snd_seq_port_subscribe *info) {
  struct snd_seq_printk_dev *rdev;
  rdev = private_data;
  if (!try_module_get(rdev->card->module))
    return -EFAULT;
  rdev->flags |= SNDRV_PRINTKMIDI_USE;
  return 0;
}

static int printk_unuse(void *private_data, struct snd_seq_port_subscribe *info) {
  struct snd_seq_printk_dev *rdev;
  rdev = private_data;
  rdev->flags &= ~SNDRV_PRINTKMIDI_USE;
  module_put(rdev->card->module);
  return 0;
}


/*
 * create a port
 */
static struct snd_seq_printk_dev *create_port( int client, int type ) {
	 struct snd_seq_port_info pinfo;
	 struct snd_seq_port_callback pcb;
	 struct snd_seq_printk_dev *rdev;
	 if ((rdev = kzalloc(sizeof(*rdev), GFP_KERNEL)) == NULL) return NULL;
		rdev->client = client;
		memset(&pinfo, 0, sizeof(pinfo));
		pinfo.addr.client = client;
  strcpy( pinfo.name, "printk" );
		pinfo.capability = SNDRV_SEQ_PORT_CAP_READ | SNDRV_SEQ_PORT_CAP_SUBS_READ;
		pinfo.capability |= SNDRV_SEQ_PORT_CAP_WRITE | SNDRV_SEQ_PORT_CAP_SUBS_WRITE;
  pinfo.capability |= SNDRV_SEQ_PORT_CAP_DUPLEX;
  pinfo.type = SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC
     | SNDRV_SEQ_PORT_TYPE_SOFTWARE
     | SNDRV_SEQ_PORT_TYPE_PORT;
  pinfo.midi_channels = 16;
		memset(&pcb, 0, sizeof(pcb));
		pcb.owner = THIS_MODULE;
  pcb.subscribe = printk_subscribe;
  pcb.unsubscribe = printk_unsubscribe;
  pcb.use = printk_use;
  pcb.unuse = printk_unuse;
  pcb.event_input = printk_input;
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

static int snd_printkmidi_probe( struct platform_device *devptr ) {
  struct snd_card *card;
  struct snd_printk *printk_;
  struct snd_seq_printk_dev *rdev;
  int err;
  int client;
  card = NULL;
  err = snd_card_new(
    &devptr->dev, -1, NULL, THIS_MODULE, sizeof(struct snd_printk ), &card
  );
  if( err < 0 ) {
    dev_err( &devptr->dev, "Unable to create snd_card: %d\n", err );
    return err;
  }
  printk_ = card->private_data;
  printk_->card = card;

 	/* create client */
	 client = snd_seq_create_kernel_client( card, 0, "printk" );
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
  printk_->rdev = rdev;
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
  rdev->client = client;
  rdev->chset->client = client;
  rdev->chset->private_data = rdev;

  strcpy(card->driver, "printk");
  strcpy(card->shortname, "printk");
  sprintf(card->longname, "printk Card %i", devptr->id + 1);
  rdev->work_queue = create_workqueue("printkmidi_queue");
  INIT_WORK( (struct work_struct *)rdev, snd_printk_worker );
  err = snd_card_register(card);
  if (!err) {
    platform_set_drvdata(devptr, card);
    return 0;
  }
  kfree( rdev );
	 snd_seq_delete_kernel_client( client );
  snd_card_free(card);
  return err;
}

static int snd_printkmidi_remove(struct platform_device *devptr) {
  struct snd_card *card;
  struct snd_printk *printk_;
  struct snd_seq_printk_dev *rdev;
  card = platform_get_drvdata(devptr);
  if( card ) {
    printk_ = card->private_data;
    if( printk_ ) {
      rdev = printk_->rdev;
      if( rdev ) {
        flush_workqueue( rdev->work_queue );
        destroy_workqueue( rdev->work_queue );
        snd_midi_channel_free_set( rdev->chset );
	       snd_seq_delete_kernel_client( rdev->client );
	       kfree( rdev );
      }
    }
    snd_card_free( card );
  }
  return 0;
}

static struct platform_driver snd_printkmidi_driver = {
  .probe  = snd_printkmidi_probe,
  .remove  = snd_printkmidi_remove,
  .driver  = {
    .name = "snd_printk",
  },
};

static struct platform_device *device;

static void snd_printkmidi_unregister_all(void) {
  platform_device_unregister(device);
  device = NULL;
  platform_driver_unregister(&snd_printkmidi_driver);
}


static int __init alsa_card_printkmidi_init(void) {
	 int err;
		struct platform_device *device_;
	 err = platform_driver_register(&snd_printkmidi_driver);
	 if (err < 0) return err;

	 device_ = platform_device_register_simple( "snd_printk", 0, NULL, 0);
 	if (IS_ERR(device_)) {
    device = NULL;
#ifdef MODULE
  		printk(KERN_ERR "Card-printkMIDI soundcard not found or device busy 1\n");
#endif
	  	snd_printkmidi_unregister_all();
	 	 return -ENODEV;
  }
	 if (!platform_get_drvdata(device_)) {
    platform_device_unregister(device_);
    device = NULL;
#ifdef MODULE
    printk(KERN_ERR "Card-printkMIDI soundcard not found or device busy\n");
#endif
	 	 snd_printkmidi_unregister_all();
		  return -ENODEV;
	 }
  device = device_;
	 return 0;
}

static void __exit alsa_card_printkmidi_exit(void) {
  snd_printkmidi_unregister_all();
}

module_init(alsa_card_printkmidi_init);
module_exit(alsa_card_printkmidi_exit);

MODULE_AUTHOR("Naomasa Matsubayashi <>");
MODULE_DESCRIPTION("printk Client Driver");
MODULE_LICENSE("GPL");

