#ifndef LPC1114_RECEIVE_BUFFER_HPP
#define LPC1114_RECEIVE_BUFFER_HPP

struct receive_buffer {
  using value_type = uint8_t;
  constexpr static size_t buffer_size = 32;
public:
  receive_buffer() : head( buffer ), tail( buffer ), buffer_end( buffer + buffer_size ) {
    std::fill( buffer, buffer + buffer_size, 0u );
  }
  receive_buffer( const receive_buffer &src ) :
    head( buffer + ( src.head - src.buffer ) ),
    tail( buffer + ( src.tail - src.buffer ) ),
    buffer_end( buffer + buffer_size ) {
    std::copy( src.buffer, src.buffer + buffer_size, buffer );
  }
  receive_buffer operator=( const receive_buffer &src ) {
    head = buffer + ( src.head - src.buffer );
    tail = buffer + ( src.tail - src.buffer );
    std::copy( src.buffer, src.buffer + buffer_size, buffer );
    return *this;
  }
  bool push( value_type value ) {
    auto next_head = next( head );
    if( next_head == tail ) return false;
    *head = value;
    head = next_head;
    return true;
  }
  value_type pop() {
    if( empty() ) return 0u;
    value_type value = *tail;
    tail = next( tail );
    return value;
  }
  bool empty() const {
    return head == tail;
  }
private:
  value_type *next( value_type *iter ) {
    ++iter;
    if( iter == buffer_end ) iter = buffer;
    return iter;
  }
  value_type * volatile head;
  value_type * volatile tail;
  value_type buffer[ buffer_size ];
  value_type * volatile buffer_end;
};

#endif
