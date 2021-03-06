"""autogenerated by genpy from rgbd_vro/vroResult.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class vroResult(genpy.Message):
  _md5sum = "ae660177fbc72eff1c9b3b4093bff3e8"
  _type = "rgbd_vro/vroResult"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#result definition
float32[] translation   # [3] : x, y, z 
float32[] orientation  # [3] : rx, ry, rz
float32[] comp_time  # [8] : load1, detection1, description1, load2, detection2, description2, match, ransac
int32[] match_num # [2] : match_num, ransac_num
int32 error
float64 depth_file_name_number

"""
  __slots__ = ['translation','orientation','comp_time','match_num','error','depth_file_name_number']
  _slot_types = ['float32[]','float32[]','float32[]','int32[]','int32','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       translation,orientation,comp_time,match_num,error,depth_file_name_number

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(vroResult, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.translation is None:
        self.translation = []
      if self.orientation is None:
        self.orientation = []
      if self.comp_time is None:
        self.comp_time = []
      if self.match_num is None:
        self.match_num = []
      if self.error is None:
        self.error = 0
      if self.depth_file_name_number is None:
        self.depth_file_name_number = 0.
    else:
      self.translation = []
      self.orientation = []
      self.comp_time = []
      self.match_num = []
      self.error = 0
      self.depth_file_name_number = 0.

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
      length = len(self.translation)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.translation))
      length = len(self.orientation)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.orientation))
      length = len(self.comp_time)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.comp_time))
      length = len(self.match_num)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.match_num))
      _x = self
      buff.write(_struct_id.pack(_x.error, _x.depth_file_name_number))
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
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.translation = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.orientation = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.comp_time = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.match_num = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 12
      (_x.error, _x.depth_file_name_number,) = _struct_id.unpack(str[start:end])
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
      length = len(self.translation)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.translation.tostring())
      length = len(self.orientation)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.orientation.tostring())
      length = len(self.comp_time)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.comp_time.tostring())
      length = len(self.match_num)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.match_num.tostring())
      _x = self
      buff.write(_struct_id.pack(_x.error, _x.depth_file_name_number))
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
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.translation = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.orientation = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.comp_time = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.match_num = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      _x = self
      start = end
      end += 12
      (_x.error, _x.depth_file_name_number,) = _struct_id.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_id = struct.Struct("<id")
