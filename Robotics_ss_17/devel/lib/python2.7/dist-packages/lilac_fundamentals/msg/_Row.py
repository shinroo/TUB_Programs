"""autogenerated by genpy from lilac_fundamentals/Row.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import lilac_fundamentals.msg

class Row(genpy.Message):
  _md5sum = "7c660e46d29bbf9fcdd244a209ce096c"
  _type = "lilac_fundamentals/Row"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Cell[] cells

================================================================================
MSG: lilac_fundamentals/Cell
# constants for walls
int32 RIGHT = 0
int32 TOP = 1
int32 LEFT = 2
int32 BOTTOM = 3
# walls that are present in this cell
int32[] walls

"""
  __slots__ = ['cells']
  _slot_types = ['lilac_fundamentals/Cell[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       cells

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Row, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.cells is None:
        self.cells = []
    else:
      self.cells = []

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
      length = len(self.cells)
      buff.write(_struct_I.pack(length))
      for val1 in self.cells:
        length = len(val1.walls)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(struct.pack(pattern, *val1.walls))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.cells is None:
        self.cells = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cells = []
      for i in range(0, length):
        val1 = lilac_fundamentals.msg.Cell()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        end += struct.calcsize(pattern)
        val1.walls = struct.unpack(pattern, str[start:end])
        self.cells.append(val1)
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
      length = len(self.cells)
      buff.write(_struct_I.pack(length))
      for val1 in self.cells:
        length = len(val1.walls)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(val1.walls.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.cells is None:
        self.cells = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cells = []
      for i in range(0, length):
        val1 = lilac_fundamentals.msg.Cell()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        end += struct.calcsize(pattern)
        val1.walls = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
        self.cells.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
