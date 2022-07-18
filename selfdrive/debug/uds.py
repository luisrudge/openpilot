from typing import List, Optional, Set

import panda.python.uds as uds

SERVICE_TYPE = uds.SERVICE_TYPE
SERVICE_TYPES = set(item.value for item in SERVICE_TYPE)
SESSION_TYPE = uds.SESSION_TYPE
SESSION_TYPES = set(item.value for item in SESSION_TYPE)
RESET_TYPE = uds.RESET_TYPE
RESET_TYPES = set(item.value for item in RESET_TYPE)
ACCESS_TYPE = uds.ACCESS_TYPE
ACCESS_TYPES = set(item.value for item in ACCESS_TYPE)
DATA_IDENTIFIER_TYPE = uds.DATA_IDENTIFIER_TYPE
DATA_IDENTIFIER_TYPES = set(item.value for item in DATA_IDENTIFIER_TYPE)
NEGATIVE_RESPONSE_CODES = uds._negative_response_codes


def hexify(data: bytes) -> str:
  return ''.join(f"\\x{b:02x}" for b in data)


def snake_to_camel(s: str) -> str:
  return ''.join(c for c in s.title() if c != '_')


class UDSPacket():
  def __init__(self, data: bytes, srcs: Set[int]) -> None:
    assert len(data) >= 1
    self.data = data[1:]
    self.service_identifier = data[0]
    self.srcs = srcs

  @property
  def name(self) -> str:
    return snake_to_camel(self.service_type.name) if self.service_type else "UDSPacket"

  @property
  def service_type(self) -> Optional[SERVICE_TYPE]:
    sid = self.service_identifier & (0xFF - 0x40)
    return SERVICE_TYPE(sid) if sid in SERVICE_TYPES else None

  @property
  def is_response(self) -> bool:
    return self.service_identifier & 0x40 == 0x40

  def _dict_to_str(self, d: dict) -> str:
    s = ""
    for k, v in d.items():
      s += f"{k}={v}, "
    return s[:-2]

  def _build_str(self, props: dict = dict()) -> str:  # pylint: disable=dangerous-default-value
    if self.name == "UDSPacket":
      props = {
        "sid": f"{self.service_identifier:#02x}",
        "data": hexify(self.data),
      }
    default_props = {
      "srcs": self.srcs,
    }
    props_str = self._dict_to_str({**props, **default_props})
    return f"{self.name}{'.PositiveResponse' if self.is_response else '.Request'}({props_str})"

  def __str__(self) -> str:
    return self._build_str()

  def __eq__(self, __o: object) -> bool:
    return isinstance(__o, UDSPacket) and self.service_identifier == __o.service_identifier and self.data == __o.data


class UDSService:
  class Request(UDSPacket):
    pass

  class Response(UDSPacket):
    pass


class DiagnosticSessionControlService(UDSService):
  """
  DIAGNOSTIC_SESSION_CONTROL (0x10)

  Parameters:
    Session:
      Allowed values are from 0 to 0x7F.
      0x01: Default session
      0x02: Programming session
      0x03: Extended diagnostic session
      0x04: Safety system diagnostic session
  """

  class Request(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      assert len(data) == 2
      super().__init__(data, srcs)

    @property
    def session_id(self) -> int:
      return self.data[0]

    @property
    def session_type(self) -> Optional[SESSION_TYPE]:
      return SESSION_TYPE(self.session_id) if self.session_id in SESSION_TYPES else None

    def __str__(self) -> str:
      return self._build_str({
        "session_type": self.session_type.name if self.session_type is not None else f"{self.session_id:#04x}",
      })

  class Response(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      super().__init__(data, srcs)

    @property
    def session_id_echo(self) -> int:
      return self.data[0]

    @property
    def session_type_echo(self) -> Optional[SESSION_TYPE]:
      return SESSION_TYPE(self.session_id_echo) if self.session_id_echo in SESSION_TYPES else None

    def __str__(self) -> str:
      return self._build_str({
        "session_type_echo": self.session_type_echo.name if self.session_type_echo is not None else f"{self.session_id_echo:#04x}",
      })


class ECUResetService(UDSService):
  """
  ECU_RESET (0x11)

  Parameters:
    ResetType:
      Allowed values are from 0 to 0x7F.
      0x01: Hard reset
      0x02: Key off on reset
      0x03: Soft reset
      0x04: Enable rapid power shutdown
      0x05: Disable rapid power shutdown
  """

  class Request(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      assert len(data) == 2
      super().__init__(data, srcs)

    @property
    def reset_id(self) -> int:
      return self.data[0]

    @property
    def reset_type(self) -> Optional[RESET_TYPE]:
      return RESET_TYPE(self.reset_id) if self.reset_id in RESET_TYPES else None

    def __str__(self) -> str:
      return self._build_str({
        "reset_type": self.reset_type.name if self.reset_type is not None else f"{self.reset_id:#02x}",
      })

  class Response(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      super().__init__(data, srcs)

    @property
    def reset_id_echo(self) -> int:
      return self.data[0]

    @property
    def reset_type_echo(self) -> Optional[RESET_TYPE]:
      return RESET_TYPE(self.reset_id_echo) if self.reset_id_echo in RESET_TYPES else None

    def __str__(self) -> str:
      return self._build_str({
        "reset_type_echo": self.reset_type_echo.name if self.reset_type_echo is not None else f"{self.reset_id_echo:#02x}",
      })


class SecurityAccessService(UDSService):
  """
  SECURITY_ACCESS (0x27)

  Parameters:
    Level:

  """

  class Request(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      super().__init__(data, srcs)

    @property
    def level(self) -> int:
      return self.data[0]

    @property
    def access_type(self) -> ACCESS_TYPE:
      return ACCESS_TYPE.SEND_KEY if self.level % 2 == 0 else ACCESS_TYPE.REQUEST_SEED

    @property
    def security_data(self) -> bytes:
      if self.access_type == ACCESS_TYPE.SEND_KEY:
        # Security key is required for send key
        return self.data[1:]
      elif self.access_type == ACCESS_TYPE.REQUEST_SEED:
        # Security access data record is optional for request seed
        return self.data[1:] if len(self.data) > 1 else b""

    def __str__(self) -> str:
      params = {
        "level": f"{self.level:#02x}",
        "mode": self.access_type.name,
      }
      if self.access_type == ACCESS_TYPE.SEND_KEY:
        params["key"] = f"\"{hexify(self.security_data)}\""
      elif self.access_type == ACCESS_TYPE.REQUEST_SEED:
        params["record"] = f"\"{hexify(self.security_data)}\""
      return self._build_str(params)

  class Response(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      super().__init__(data, srcs)

    @property
    def level(self) -> int:
      return self.data[0]

    @property
    def access_type(self) -> ACCESS_TYPE:
      return ACCESS_TYPE.SEND_KEY if self.level % 2 == 0 else ACCESS_TYPE.REQUEST_SEED

    @property
    def seed(self) -> bytes:
      return self.data[1:] if self.access_type == ACCESS_TYPE.REQUEST_SEED else b""

    def __str__(self) -> str:
      params = {
        "level": f"{self.level:#02x}",
        "mode": self.access_type.name,
      }
      if self.access_type == ACCESS_TYPE.REQUEST_SEED:
        params["seed"] = f"\"{hexify(self.seed)}\""
      return self._build_str(params)


def _did_to_desc(did: int) -> str:
  return f"{DATA_IDENTIFIER_TYPE(did).name}" if did in DATA_IDENTIFIER_TYPES else f"{did:#04x}"


def _didlist_to_desc(didlist: List[int]) -> List[str]:
  return [_did_to_desc(did) for did in didlist]


class ReadDataByIdentifierService(UDSService):
  class Request(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      super().__init__(data, srcs)

    @property
    def didlist(self) -> List[int]:
      dids = []
      for i in range(0, len(self.data), 2):
        if i + 1 == len(self.data):
          break
        dids.append((self.data[i] << 8) + (self.data[i + 1] << 0))
      return dids

    def __str__(self) -> str:
      return self._build_str({
        "didlist": f"{_didlist_to_desc(self.didlist)}",
      })

  class Response(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      super().__init__(data, srcs)

    def __str__(self) -> str:
      # TODO: implement for more than one value
      value = self.data[2:]
      return self._build_str({
        "raw": f"{hexify(value)}",
        "utf8": f"\"{value.decode('utf-8', errors='replace')}\"",
      })


class WriteDataByIdentifierService(UDSService):
  class Request(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      super().__init__(data, srcs)

    @property
    def did(self) -> int:
      return (self.data[0] << 8) + (self.data[1] << 0)

    @property
    def value(self) -> bytes:
      return self.data[2:]

    def __str__(self) -> str:
      return self._build_str({
        "did": _did_to_desc(self.did),
        "value": f"\"{hexify(self.value)}\"",
      })

  class Response(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      super().__init__(data, srcs)

    @property
    def did_echo(self) -> int:
      return (self.data[0] << 8) + (self.data[1] << 0)

    def __str__(self) -> str:
      return self._build_str({
        "did_echo": _did_to_desc(self.did_echo),
      })


class ClearDiagnosticInformationService(UDSService):
  class Request(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      super().__init__(data, srcs)

    @property
    def group(self) -> int:
      """
      DTC mask ranging from 0 to 0xFFFFFF. 0xFFFFFF means all DTCs.
      """
      return self.data[0] << 16 | self.data[1] << 8 | self.data[2]

    def __str__(self) -> str:
      return self._build_str({
        "group": f"{self.group:#06x}",
      })

  class Response(UDSPacket):
    def __init__(self, data: bytes, srcs: Set[int]):
      super().__init__(data, srcs)


class NegativeResponse(UDSPacket):
  def __init__(self, data: bytes, srcs: Set[int]):
    super().__init__(data, srcs)

  @property
  def negative_response_code_id(self) -> int:
    return self.data[0]

  @property
  def negative_response_code_desc(self) -> Optional[str]:
    return NEGATIVE_RESPONSE_CODES.get(self.negative_response_code_id)

  def __str__(self) -> str:
    props = {
      "code": f"\"{self.negative_response_code_desc}\"",
    }
    if self.service_type is None:
      props["service_id"] = f"{self.service_identifier:#04x}"
    return f"{self.name}.NegativeResponse({self._dict_to_str(props)})"


UDS_SERVICES = {
  SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL: DiagnosticSessionControlService,
  SERVICE_TYPE.ECU_RESET: ECUResetService,
  SERVICE_TYPE.SECURITY_ACCESS: SecurityAccessService,
  SERVICE_TYPE.READ_DATA_BY_IDENTIFIER: ReadDataByIdentifierService,
  SERVICE_TYPE.WRITE_DATA_BY_IDENTIFIER: WriteDataByIdentifierService,
  SERVICE_TYPE.CLEAR_DIAGNOSTIC_INFORMATION: ClearDiagnosticInformationService,
}


def parse_uds_packet(data: bytes, srcs: Set[int]) -> Optional[UDSPacket]:
  service_identifier = int(data[0])

  # Handle negative response
  if service_identifier == 0x7F:
    data = data[1:]
    return NegativeResponse(data, srcs)

  else:
    request = True
    if service_identifier & 0x40 == 0x40:
      request = False
      service_identifier -= 0x40

    service = UDSService
    try:
      service_type = SERVICE_TYPE(service_identifier)
      if service_type in UDS_SERVICES:
        service = UDS_SERVICES[service_type]
    except ValueError:
      pass

    if request:
      return service.Request(data, srcs)
    else:
      return service.Response(data, srcs)
