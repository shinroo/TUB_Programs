"""autogenerated by genpy from lilac_fundamentals/ActualLocalization.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ActualLocalization(genpy.Message):
  _md5sum = "893e067a4f69fb209f2b2e037a5aa78b"
  _type = "lilac_fundamentals/ActualLocalization"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int16 row
int16 column
int16 orientation
bool localized

"""
  __slots__ = ['row','column','orientation','localized']
  _slot_types = ['int16','int16','int16','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       row,column,orientation,localized

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ActualLocalization, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.row is None:
        self.row = 0
      if self.column is None:
        self.column = 0
      if self.orientation is None:
        self.orientation = 0
      if self.localized is None:
        self.localized = False
    else:
      self.row = 0
      self.column = 0
      self.orientation = 0
      self.localized = False

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
      buff.write(_struct_3hB.pack(_x.row, _x.column, _x.orientation, _x.localized))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 7
      (_x.row, _x.column, _x.orientation, _x.localized,) = _struct_3hB.unpack(str[start:end])
      self.localized = bool(self.localized)
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
      buff.write(_struct_3hB.pack(_x.row, _x.column, _x.orientation, _x.localized))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 7
      (_x.row, _x.column, _x.orientation, _x.localized,) = _struct_3hB.unpack(str[start:end])
      self.localized = bool(self.localized)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3hB = struct.Struct("<3hB")