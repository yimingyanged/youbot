"""autogenerated by genpy from image_cache/GetImageRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class GetImageRequest(genpy.Message):
  _md5sum = "a1ec92a8ece4f3e75b4d829c9a3a2dd4"
  _type = "image_cache/GetImageRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
int32 from
int32[] to
bool rectified

"""
  __slots__ = ['from_','to','rectified']
  _slot_types = ['int32','int32[]','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       from_,to,rectified

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetImageRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.from_ is None:
        self.from_ = 0
      if self.to is None:
        self.to = []
      if self.rectified is None:
        self.rectified = False
    else:
      self.from_ = 0
      self.to = []
      self.rectified = False

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
      buff.write(_struct_i.pack(self.from_))
      length = len(self.to)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.to))
      buff.write(_struct_B.pack(self.rectified))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (self.from_,) = _struct_i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.to = struct.unpack(pattern, str[start:end])
      start = end
      end += 1
      (self.rectified,) = _struct_B.unpack(str[start:end])
      self.rectified = bool(self.rectified)
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
      buff.write(_struct_i.pack(self.from_))
      length = len(self.to)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.to.tostring())
      buff.write(_struct_B.pack(self.rectified))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (self.from_,) = _struct_i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.to = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 1
      (self.rectified,) = _struct_B.unpack(str[start:end])
      self.rectified = bool(self.rectified)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_i = struct.Struct("<i")
_struct_B = struct.Struct("<B")
"""autogenerated by genpy from image_cache/GetImageResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import sensor_msgs.msg

class GetImageResponse(genpy.Message):
  _md5sum = "715532082a0455e476f20b354f3beb2a"
  _type = "image_cache/GetImageResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 from_id
int32[] to_id
sensor_msgs/Image from_l
sensor_msgs/Image from_r
sensor_msgs/Image[] to_l
sensor_msgs/Image[] to_r


================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['from_id','to_id','from_l','from_r','to_l','to_r']
  _slot_types = ['int32','int32[]','sensor_msgs/Image','sensor_msgs/Image','sensor_msgs/Image[]','sensor_msgs/Image[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       from_id,to_id,from_l,from_r,to_l,to_r

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetImageResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.from_id is None:
        self.from_id = 0
      if self.to_id is None:
        self.to_id = []
      if self.from_l is None:
        self.from_l = sensor_msgs.msg.Image()
      if self.from_r is None:
        self.from_r = sensor_msgs.msg.Image()
      if self.to_l is None:
        self.to_l = []
      if self.to_r is None:
        self.to_r = []
    else:
      self.from_id = 0
      self.to_id = []
      self.from_l = sensor_msgs.msg.Image()
      self.from_r = sensor_msgs.msg.Image()
      self.to_l = []
      self.to_r = []

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
      buff.write(_struct_i.pack(self.from_id))
      length = len(self.to_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.to_id))
      _x = self
      buff.write(_struct_3I.pack(_x.from_l.header.seq, _x.from_l.header.stamp.secs, _x.from_l.header.stamp.nsecs))
      _x = self.from_l.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.from_l.height, _x.from_l.width))
      _x = self.from_l.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.from_l.is_bigendian, _x.from_l.step))
      _x = self.from_l.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.from_r.header.seq, _x.from_r.header.stamp.secs, _x.from_r.header.stamp.nsecs))
      _x = self.from_r.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.from_r.height, _x.from_r.width))
      _x = self.from_r.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.from_r.is_bigendian, _x.from_r.step))
      _x = self.from_r.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.to_l)
      buff.write(_struct_I.pack(length))
      for val1 in self.to_l:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2I.pack(_x.height, _x.width))
        _x = val1.encoding
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_BI.pack(_x.is_bigendian, _x.step))
        _x = val1.data
        length = len(_x)
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(struct.pack('<I%sB'%length, length, *_x))
        else:
          buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.to_r)
      buff.write(_struct_I.pack(length))
      for val1 in self.to_r:
        _v3 = val1.header
        buff.write(_struct_I.pack(_v3.seq))
        _v4 = _v3.stamp
        _x = _v4
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v3.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2I.pack(_x.height, _x.width))
        _x = val1.encoding
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_BI.pack(_x.is_bigendian, _x.step))
        _x = val1.data
        length = len(_x)
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(struct.pack('<I%sB'%length, length, *_x))
        else:
          buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.from_l is None:
        self.from_l = sensor_msgs.msg.Image()
      if self.from_r is None:
        self.from_r = sensor_msgs.msg.Image()
      if self.to_l is None:
        self.to_l = None
      if self.to_r is None:
        self.to_r = None
      end = 0
      start = end
      end += 4
      (self.from_id,) = _struct_i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.to_id = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 12
      (_x.from_l.header.seq, _x.from_l.header.stamp.secs, _x.from_l.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.from_l.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.from_l.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.from_l.height, _x.from_l.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.from_l.encoding = str[start:end].decode('utf-8')
      else:
        self.from_l.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.from_l.is_bigendian, _x.from_l.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.from_l.data = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.from_r.header.seq, _x.from_r.header.stamp.secs, _x.from_r.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.from_r.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.from_r.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.from_r.height, _x.from_r.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.from_r.encoding = str[start:end].decode('utf-8')
      else:
        self.from_r.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.from_r.is_bigendian, _x.from_r.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.from_r.data = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.to_l = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.Image()
        _v5 = val1.header
        start = end
        end += 4
        (_v5.seq,) = _struct_I.unpack(str[start:end])
        _v6 = _v5.stamp
        _x = _v6
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v5.frame_id = str[start:end].decode('utf-8')
        else:
          _v5.frame_id = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.height, _x.width,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.encoding = str[start:end].decode('utf-8')
        else:
          val1.encoding = str[start:end]
        _x = val1
        start = end
        end += 5
        (_x.is_bigendian, _x.step,) = _struct_BI.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.data = str[start:end]
        self.to_l.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.to_r = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.Image()
        _v7 = val1.header
        start = end
        end += 4
        (_v7.seq,) = _struct_I.unpack(str[start:end])
        _v8 = _v7.stamp
        _x = _v8
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v7.frame_id = str[start:end].decode('utf-8')
        else:
          _v7.frame_id = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.height, _x.width,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.encoding = str[start:end].decode('utf-8')
        else:
          val1.encoding = str[start:end]
        _x = val1
        start = end
        end += 5
        (_x.is_bigendian, _x.step,) = _struct_BI.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.data = str[start:end]
        self.to_r.append(val1)
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
      buff.write(_struct_i.pack(self.from_id))
      length = len(self.to_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.to_id.tostring())
      _x = self
      buff.write(_struct_3I.pack(_x.from_l.header.seq, _x.from_l.header.stamp.secs, _x.from_l.header.stamp.nsecs))
      _x = self.from_l.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.from_l.height, _x.from_l.width))
      _x = self.from_l.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.from_l.is_bigendian, _x.from_l.step))
      _x = self.from_l.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.from_r.header.seq, _x.from_r.header.stamp.secs, _x.from_r.header.stamp.nsecs))
      _x = self.from_r.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.from_r.height, _x.from_r.width))
      _x = self.from_r.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.from_r.is_bigendian, _x.from_r.step))
      _x = self.from_r.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.to_l)
      buff.write(_struct_I.pack(length))
      for val1 in self.to_l:
        _v9 = val1.header
        buff.write(_struct_I.pack(_v9.seq))
        _v10 = _v9.stamp
        _x = _v10
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v9.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2I.pack(_x.height, _x.width))
        _x = val1.encoding
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_BI.pack(_x.is_bigendian, _x.step))
        _x = val1.data
        length = len(_x)
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(struct.pack('<I%sB'%length, length, *_x))
        else:
          buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.to_r)
      buff.write(_struct_I.pack(length))
      for val1 in self.to_r:
        _v11 = val1.header
        buff.write(_struct_I.pack(_v11.seq))
        _v12 = _v11.stamp
        _x = _v12
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v11.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2I.pack(_x.height, _x.width))
        _x = val1.encoding
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_BI.pack(_x.is_bigendian, _x.step))
        _x = val1.data
        length = len(_x)
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(struct.pack('<I%sB'%length, length, *_x))
        else:
          buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.from_l is None:
        self.from_l = sensor_msgs.msg.Image()
      if self.from_r is None:
        self.from_r = sensor_msgs.msg.Image()
      if self.to_l is None:
        self.to_l = None
      if self.to_r is None:
        self.to_r = None
      end = 0
      start = end
      end += 4
      (self.from_id,) = _struct_i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.to_id = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      _x = self
      start = end
      end += 12
      (_x.from_l.header.seq, _x.from_l.header.stamp.secs, _x.from_l.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.from_l.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.from_l.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.from_l.height, _x.from_l.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.from_l.encoding = str[start:end].decode('utf-8')
      else:
        self.from_l.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.from_l.is_bigendian, _x.from_l.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.from_l.data = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.from_r.header.seq, _x.from_r.header.stamp.secs, _x.from_r.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.from_r.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.from_r.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.from_r.height, _x.from_r.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.from_r.encoding = str[start:end].decode('utf-8')
      else:
        self.from_r.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.from_r.is_bigendian, _x.from_r.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.from_r.data = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.to_l = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.Image()
        _v13 = val1.header
        start = end
        end += 4
        (_v13.seq,) = _struct_I.unpack(str[start:end])
        _v14 = _v13.stamp
        _x = _v14
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v13.frame_id = str[start:end].decode('utf-8')
        else:
          _v13.frame_id = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.height, _x.width,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.encoding = str[start:end].decode('utf-8')
        else:
          val1.encoding = str[start:end]
        _x = val1
        start = end
        end += 5
        (_x.is_bigendian, _x.step,) = _struct_BI.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.data = str[start:end]
        self.to_l.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.to_r = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.Image()
        _v15 = val1.header
        start = end
        end += 4
        (_v15.seq,) = _struct_I.unpack(str[start:end])
        _v16 = _v15.stamp
        _x = _v16
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v15.frame_id = str[start:end].decode('utf-8')
        else:
          _v15.frame_id = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.height, _x.width,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.encoding = str[start:end].decode('utf-8')
        else:
          val1.encoding = str[start:end]
        _x = val1
        start = end
        end += 5
        (_x.is_bigendian, _x.step,) = _struct_BI.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.data = str[start:end]
        self.to_r.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_i = struct.Struct("<i")
_struct_3I = struct.Struct("<3I")
_struct_2I = struct.Struct("<2I")
_struct_BI = struct.Struct("<BI")
class GetImage(object):
  _type          = 'image_cache/GetImage'
  _md5sum = '977c89c0bbd082bbbbc98818a5de36b1'
  _request_class  = GetImageRequest
  _response_class = GetImageResponse
