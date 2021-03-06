# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from lama_msgs/InterfaceInfo.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class InterfaceInfo(genpy.Message):
  _md5sum = "156de5bd66b7a123b19f5a2142f26e8c"
  _type = "lama_msgs/InterfaceInfo"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# Interface name.
string interface_name

# The type of message associated with the interface.
string message_type

# The name of the associated service for getting a message.
string get_service_name

# The service message of the service for getting a message.
string get_service_type

# The name of the associated service for setting a message.
string set_service_name

# The service message of the service for setting a message.
string set_service_type

string interface_type

# Timestamp of the last added message through this interface.
time timestamp
"""
  __slots__ = ['interface_name','message_type','get_service_name','get_service_type','set_service_name','set_service_type','interface_type','timestamp']
  _slot_types = ['string','string','string','string','string','string','string','time']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       interface_name,message_type,get_service_name,get_service_type,set_service_name,set_service_type,interface_type,timestamp

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(InterfaceInfo, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.interface_name is None:
        self.interface_name = ''
      if self.message_type is None:
        self.message_type = ''
      if self.get_service_name is None:
        self.get_service_name = ''
      if self.get_service_type is None:
        self.get_service_type = ''
      if self.set_service_name is None:
        self.set_service_name = ''
      if self.set_service_type is None:
        self.set_service_type = ''
      if self.interface_type is None:
        self.interface_type = ''
      if self.timestamp is None:
        self.timestamp = genpy.Time()
    else:
      self.interface_name = ''
      self.message_type = ''
      self.get_service_name = ''
      self.get_service_type = ''
      self.set_service_name = ''
      self.set_service_type = ''
      self.interface_type = ''
      self.timestamp = genpy.Time()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.interface_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.message_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.get_service_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.get_service_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.set_service_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.set_service_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.interface_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.timestamp.secs, _x.timestamp.nsecs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.interface_name = str[start:end].decode('utf-8')
      else:
        self.interface_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.message_type = str[start:end].decode('utf-8')
      else:
        self.message_type = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.get_service_name = str[start:end].decode('utf-8')
      else:
        self.get_service_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.get_service_type = str[start:end].decode('utf-8')
      else:
        self.get_service_type = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.set_service_name = str[start:end].decode('utf-8')
      else:
        self.set_service_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.set_service_type = str[start:end].decode('utf-8')
      else:
        self.set_service_type = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.interface_type = str[start:end].decode('utf-8')
      else:
        self.interface_type = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.timestamp.secs, _x.timestamp.nsecs,) = _get_struct_2I().unpack(str[start:end])
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.interface_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.message_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.get_service_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.get_service_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.set_service_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.set_service_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.interface_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.timestamp.secs, _x.timestamp.nsecs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.interface_name = str[start:end].decode('utf-8')
      else:
        self.interface_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.message_type = str[start:end].decode('utf-8')
      else:
        self.message_type = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.get_service_name = str[start:end].decode('utf-8')
      else:
        self.get_service_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.get_service_type = str[start:end].decode('utf-8')
      else:
        self.get_service_type = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.set_service_name = str[start:end].decode('utf-8')
      else:
        self.set_service_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.set_service_type = str[start:end].decode('utf-8')
      else:
        self.set_service_type = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.interface_type = str[start:end].decode('utf-8')
      else:
        self.interface_type = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.timestamp.secs, _x.timestamp.nsecs,) = _get_struct_2I().unpack(str[start:end])
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
