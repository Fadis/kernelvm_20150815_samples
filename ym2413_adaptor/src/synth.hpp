#ifndef LPC1114_SYNTH_HPP
#define LPC1114_SYNTH_HPP

#include <stdexcept>
#include <algorithm>
#include "LPC11xx.h"
#include "core_cm0.h"
#include "i2c.hpp"

extern const int fnum[12];

#define DRUM_BD (1<<4)
#define DRUM_SD (1<<3)
#define DRUM_TOM (1<<2)
#define DRUM_TCY (1<<1)
#define DRUM_HH (1<<0)

extern const unsigned char programs[128];

extern const unsigned char parcussion[ 128 ]; 

struct active_note_t {
  active_note_t() : channel( 0 ), note_number( 0 ), step( 0 ) {}
  active_note_t( const active_note_t &src ) : channel( src.channel ), note_number( src.note_number ), step( src.step ) {}
  active_note_t operator=( const active_note_t &src ) {
    channel = src.channel;
    note_number = src.note_number;
    step = src.step;
    return *this;
  }
  int channel;
  int note_number;
  int step;
};

struct instrument_t {
  instrument_t() : program( 3 ), level( 127 ) {}
  instrument_t( const instrument_t &src ) : program( src.program ), level( src.level ) {}
  instrument_t &operator=( const instrument_t &src ) {
    program = src.program;
    level = src.level;
    return *this;
  }
  int program;
  int level;
};


#define CHANNEL_COUNT 6
#define MIDI_CHANNEL_COUNT 16

class channel_map_t {
public:
  typedef active_note_t value_type;
  typedef active_note_t &reference;
  typedef active_note_t *pointer;
  typedef active_note_t *iterator;
  typedef const active_note_t *const_iterator;
  typedef size_t size_type;
  typedef ptrdiff_t difference_type;
  channel_map_t() {
    std::fill( map, map + CHANNEL_COUNT, active_note_t() );
  }
  channel_map_t( const channel_map_t &src ) {
    std::copy( src.map, src.map + CHANNEL_COUNT, map );
  }
  channel_map_t &operator=( const channel_map_t &src ) {
    std::copy( src.map, src.map + CHANNEL_COUNT, map );
    return *this;
  }
  iterator begin() { return map; }
  const_iterator begin() const { return map; }
  const_iterator cbegin() const { return map; }
  iterator end() { return map + CHANNEL_COUNT; }
  const_iterator end() const { return map + CHANNEL_COUNT; }
  const_iterator cend() const { return map + CHANNEL_COUNT; }
  size_type size() const { return CHANNEL_COUNT; }
  size_type capacity() const { return CHANNEL_COUNT; }
private:
  active_note_t map[ CHANNEL_COUNT ];
};

class instruments_t {
public:
  typedef instrument_t value_type;
  typedef instrument_t &reference;
  typedef const instrument_t &const_reference;
  typedef instrument_t *pointer;
  typedef instrument_t *iterator;
  typedef const instrument_t *const_iterator;
  typedef size_t size_type;
  typedef ptrdiff_t difference_type;
  instruments_t() {
    std::fill( map, map + CHANNEL_COUNT, instrument_t() );
  }
  instruments_t( const instruments_t &src ) {
    std::copy( src.map, src.map + CHANNEL_COUNT, map );
  }
  instruments_t &operator=( const instruments_t &src ) {
    std::copy( src.map, src.map + CHANNEL_COUNT, map );
    return *this;
  }
  iterator begin() { return map; }
  const_iterator begin() const { return map; }
  const_iterator cbegin() const { return map; }
  iterator end() { return map + CHANNEL_COUNT; }
  const_iterator end() const { return map + CHANNEL_COUNT; }
  const_iterator cend() const { return map + CHANNEL_COUNT; }
  size_type size() const { return CHANNEL_COUNT; }
  size_type capacity() const { return CHANNEL_COUNT; }
  reference operator[]( size_type i ) { return map[ i ]; }
  const_reference operator[]( size_type i ) const { return map[ i ]; }
private:
  instrument_t map[ CHANNEL_COUNT ];
};


class YM2413 {
  struct is_the_channel {
    is_the_channel( int ch_, int num_ ) : ch( ch_ ), num( num_ ) {}
    bool operator()( const active_note_t &note ) const { return note.note_number == num && note.channel == ch; }
    int ch;
    int num;
  };
  enum class write_state_t {
    initial,
    init_address,
    set_address,
    hold_address,
    init_value,
    set_value,
    hold_value
  };
public:
  YM2413() : write_state( write_state_t::initial ), head( buffer ), tail( buffer ), buffer_end( buffer + 256 ), is_data( false ), drum_state( 0 ), step( 0 ) {
    LPC_GPIO0->DATA = (0<<2)|(0<<3);
    LPC_GPIO1->DATA = 0x200u;
    for( size_t count = 0; count != SystemCoreClock; ++count );
    LPC_GPIO0->DATA = (1<<2)|(0<<3);
    for( size_t count = 0; count != SystemCoreClock; ++count );
  }
  void reset() {
    head = buffer;
    tail = buffer;
    buffer_end = buffer + 256;
    is_data = false;
    drum_state = 0;
    step = 0;
    write_state = write_state_t::initial;
    LPC_GPIO0->DATA = (0<<2)|(0<<3);
    LPC_GPIO1->DATA = 0x200u;
    for( size_t count = 0; count != SystemCoreClock; ++count );
    LPC_GPIO0->DATA = (1<<2)|(0<<3);
    for( size_t count = 0; count != SystemCoreClock; ++count );
  }
  void note_on(int ch, int num, int vol) {
    if( ch != 10 ) {
      if( vol == 0 || instruments[ ch ].level == 0 )
        return;
      if( !note_on_at_blank_channel( ch, num, vol ) )
        note_on_at_oldest_channel( ch, num, vol );
    }
    else {
      int drum_val = 1 << parcussion[ num ];
      if( drum_val != ( 1 << 5 ) )
        drum( drum_val );
    }
  }
  void note_off( int ch, int num ) {
    if( ch != 10 ) {
      const channel_map_t::iterator search_result = std::find_if( channel_map.begin(), channel_map.end(), is_the_channel( ch, num ) );
      if( search_result != channel_map.end() ) {
        note_off_internal( std::distance( channel_map.begin(), search_result ) );
        search_result->note_number = 0;
      }
    }
  }
  void terminate( int ch, int num ) {
    note_off( ch, num );
  }
  void drumvol(int bd, int sd, int tom, int tcy, int hh) {
    write(0x36, bd);
    write(0x37, hh << 4 | sd);
    write(0x38, tom << 4 | tcy);
  }
  void program_change( int ch, int program ) {
    instruments[ ch ].program = program;
  }
  void pitch_bend( int, int ) {}
  void set_level( int ch, int level ) {
    instruments[ ch ].level = level;
  }
  void pop() {
    if( head != tail ) {
      if( write_state == write_state_t::initial ) {
        LPC_GPIO0->DATA = (0<<2)|(1<<3);
        LPC_GPIO1->DATA = 0x200u;
	write_state = write_state_t::init_address;
	system_state = 0x50;
      }
      else if( write_state == write_state_t::init_address ) {
        LPC_GPIO1->DATA = *tail & 0xFFu;
	write_state = write_state_t::set_address;
        system_state = 0x51;
      }
      else if( write_state == write_state_t::set_address ) {
        LPC_GPIO1->DATA = ( *tail & 0xFFu ) | 0x200u;
        ++tail;
        if( tail == buffer_end )
          tail = buffer;
	write_state = write_state_t::hold_address;
        system_state = 0x53;
      }
      else if( write_state == write_state_t::hold_address ) {
        LPC_GPIO0->DATA = (1<<2)|(1<<3);
        LPC_GPIO1->DATA = 0x200u;
	write_state = write_state_t::init_value;
        system_state = 0x54;
      }
      else if( write_state == write_state_t::init_value ) {
        LPC_GPIO1->DATA = *tail & 0xFFu;
	write_state = write_state_t::set_value;
        system_state = 0x55;
      }
      else if( write_state == write_state_t::set_value ) {
        LPC_GPIO1->DATA = ( *tail & 0xFFu ) | 0x200u;
        ++tail;
        if( tail == buffer_end )
          tail = buffer;
	write_state = write_state_t::initial;
        system_state = 0x56;
      }
    }
  }
private:
  void drum(int val) {
    write(0x0E, ( drum_state & ~val ) | ( 1 << 5 ) );
    drum_state = val | drum_state;
    write(0x0E, ( val | drum_state ) | ( 1 << 5 ) );
  }
  int get_level( int velocity, int ) {
    return 0xF - ( velocity >> 3 );
  }
  struct is_blank_channel {
    bool operator()( const active_note_t &note ) const { return note.note_number == 0; }
  };
  bool note_on_at_blank_channel(int ch, int num, int vol) {
    const channel_map_t::iterator search_result = std::find_if( channel_map.begin(), channel_map.end(), is_blank_channel() );
    if( search_result != channel_map.end() ) {
      note_on_internal( std::distance( channel_map.begin(), search_result ), num, programs[ instruments[ ch ].program ], get_level( vol, instruments[ ch ].level ) );
      search_result->channel = ch;
      search_result->note_number = num;
      search_result->step = ++step;
      return true;
    }
    return false;
  }
  struct is_older_channel {
    bool operator()( const active_note_t &left, const active_note_t &right ) const { return left.step < right.step; }
  };
  void note_on_at_oldest_channel(int ch, int num, int vol) {
    const channel_map_t::iterator search_result = std::min_element( channel_map.begin(), channel_map.end(), is_older_channel() );
    note_off_internal( std::distance( channel_map.begin(), search_result ) );
    note_on_internal( std::distance( channel_map.begin(), search_result ), num, programs[ instruments[ ch ].program ], get_level( vol, instruments[ ch ].level ) );
    search_result->channel = ch;
    search_result->note_number = num;
    search_result->step = ++step;
  }
  void note_on_internal(int ch, int num, int inst, int vol) {
    // num = 12 - 107 (MIDI Note Numbers)
    // oct = 0 - 7
    int oct;
    if (num >= 12) {num = num - 12;};
    oct = num / 12;
    if (oct >= 8) {oct = 7;};
    num = fnum[num % 12];
  
    // Note(9ch) 0x10 - 15 (0 - 5)
    write((0x10 + ch), num);

    // Inst & Vol(9ch)
    write((0x30 + ch), (inst << 4) | vol);

    // note_on & Oct & Note
    write((0x20 + ch), (16 | (oct << 1) | (num >> 8)));
  }
  
  void note_off_internal(int ch) {
    // 0x20 - 25 (0 - 5)
    write((0x20 + ch), 0);
  }
  bool push( unsigned int value ) {
    unsigned int * volatile next = head + 1;
    if( next == buffer_end )
      next = buffer;
    if( next != tail ) {
      *head = value;
      head = next;
      return true;
    }
    else
      return false;
  }
  void write( unsigned int address, unsigned int data ) {
    push( address );
    push( data );
  }
  volatile write_state_t write_state;
  unsigned int buffer[ 256 ];
  unsigned int * volatile head;
  unsigned int * volatile tail;
  unsigned int * volatile buffer_end;
  bool is_data;
  int drum_state;
  unsigned int step;
  channel_map_t channel_map;
  instruments_t instruments;
};


template< typename Receiver >
class command_state {
  enum state_t {
    initial,
    noteon_0,
    noteon_1,
    noteoff_0,
    terminate_0,
    pitchbend_0,
    pitchbend_1,
    progchange_0
  };
public:
  command_state() : state( initial ) {}
  void push( unsigned int value ) {
    value &= 0xFF;
    if( state == initial ) {
      buffer[ 0 ] = value;
      if( ( value & 0xF0 ) == 0x10 ) state = noteon_0;
      else if( ( value & 0xF0 ) == 0x20 ) state = noteoff_0;
      else if( ( value & 0xF0 ) == 0x30 ) state = terminate_0;
      else if( ( value & 0xF0 ) == 0x40 ) state = pitchbend_0;
      else if( ( value & 0xF0 ) == 0x50 ) state = progchange_0;
      else if( ( value & 0xF0 ) == 0x60 ) {}
    }
    else if( state == noteon_0 ) {
      buffer[ 1 ] = value;
      state = noteon_1;
    }
    else if( state == noteon_1 ) {
      receiver.note_on( buffer[ 0 ] & 0xF, buffer[ 1 ], value );
      state = initial;
    }
    else if( state == noteoff_0 ) {
      receiver.note_off( buffer[ 0 ] & 0xF, value );
      state = initial;
    }
    else if( state == terminate_0 ) {
      receiver.terminate( buffer[ 0 ] & 0xF, value );
      state = initial;
    }
    else if( state == pitchbend_0 ) {
      buffer[ 1 ] = value;
      state = pitchbend_1;
    }
    else if( state == pitchbend_1 ) {
      receiver.pitch_bend( buffer[ 0 ] & 0xF, ( buffer[ 1 ]| ( value << 7 ) ) - 8192 );
      state = initial;
    }
    else if( state == progchange_0 ) {
      receiver.program_change( buffer[ 0 ] & 0xF, value );
      state = initial;
    }
  }
  void reset() {
    state = initial;
    receiver.reset();
  }
  void pop() {
    receiver.pop();
  }
private:
  state_t state;
  int buffer[ 2 ];
  Receiver receiver;
};

#endif
