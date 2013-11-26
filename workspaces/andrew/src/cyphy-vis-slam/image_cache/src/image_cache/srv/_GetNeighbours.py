"""autogenerated by genpy from image_cache/GetNeighboursRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class GetNeighboursRequest(genpy.Message):
  _md5sum = "83f1d0e77402d46c26211c4c64e0f71c"
  _type = "image_cache/GetNeighboursRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
int32 nodeID

"""
  __slots__ = ['nodeID']
  _slot_types = ['int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       nodeID

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetNeighboursRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.nodeID is None:
        self.nodeID = 0
    else:
      self.nodeID = 0

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
      buff.write(_struct_i.pack(self.nodeID))
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
      (self.nodeID,) = _struct_i.unpack(str[start:end])
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
      buff.write(_struct_i.pack(self.nodeID))
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
      (self.nodeID,) = _struct_i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_i = struct.Struct("<i")
"""autogenerated by genpy from image_cache/GetNeighboursResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class GetNeighboursResponse(genpy.Message):
  _md5sum = "15fb40c1365b06d426b1f08def48bff0"
  _type = "image_cache/GetNeighboursResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 node1
int32 node2
float32 frac
time inter_time


"""
  __slots__ = ['node1','node2','frac','inter_time']
  _slot_types = ['int32','int32','float32','time']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       node1,node2,frac,inter_time

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetNeighboursResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.node1 is None:
        self.node1 = 0
      if self.node2 is None:
        self.node2 = 0
      if self.frac is None:
        self.frac = 0.
      if self.inter_time is None:
        self.inter_time = genpy.Time()
    else:
      self.node1 = 0
      self.node2 = 0
      self.frac = 0.
      self.inter_time = genpy.Time()

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
      _x = self
      buff.write(_struct_2if2I.pack(_x.node1, _x.node2, _x.frac, _x.inter_time.secs, _x.inter_time.nsecs))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.inter_time is None:
        self.inter_time = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 20
      (_x.node1, _x.node2, _x.frac, _x.inter_time.secs, _x.inter_time.nsecs,) = _struct_2if2I.unpack(str[start:end])
      self.inter_time.canon()
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
      _x = self
      buff.write(_struct_2if2I.pack(_x.node1, _x.node2, _x.frac, _x.inter_time.secs, _x.inter_time.nsecs))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.inter_time is None:
        self.inter_time = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 20
      (_x.node1, _x.node2, _x.frac, _x.inter_time.secs, _x.inter_time.nsecs,) = _struct_2if2I.unpack(str[start:end])
      self.inter_time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2if2I = struct.Struct("<2if2I")
class GetNeighbours(object):
  _type          = 'image_cache/GetNeighbours'
  _md5sum = 'ccd112ae98ddab13d42702fa6352dc06'
  _request_class  = GetNeighboursRequest
  _response_class = GetNeighboursResponse