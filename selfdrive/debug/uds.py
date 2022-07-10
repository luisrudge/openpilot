from typing import Optional

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


def hexify(data: bytes) -> str:
  return ''.join(f"\\x{b:02x}" for b in data)


def snake_to_camel(s: str) -> str:
  return ''.join(c for c in s.title() if c != '_')


class UDSPacket():
  def __init__(self, data: bytes) -> None:
    assert len(data) >= 1
    self.data = data
    self.service_identifier = data[0]

  @property
  def name(self) -> str:
    return snake_to_camel(self.service_type.name)

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

  def _build_str(self, props: dict = dict()) -> str:
    props = self._dict_to_str(props)
    return f"{self.name}{'.Response' if self.is_response else '.Request'}({props})"

  def __str__(self) -> str:
    return self._build_str()


class UDSService:
  class Request(UDSPacket):
    def __init__(self, data: bytes):
      super.__init__(data)

  class Response(UDSPacket):
    def __init__(self, data: bytes):
      super.__init__(data)


class DiagnosticSessionControlService(UDSService):
  """
  DIAGNOSTIC_SESSION_CONTROL (0x10)
  """

  class Request(UDSPacket):
    def __init__(self, data: bytes):
      assert len(data) == 2
      super().__init__(data)

    @property
    def session_id(self) -> int:
      return self.data[1]

    @property
    def session_type(self) -> Optional[SESSION_TYPE]:
      return SESSION_TYPE(self.session_id) if self.session_id in SESSION_TYPES else None

    def __str__(self) -> str:
      return self._build_str({
        "session_type": self.session_type.name if self.session_type is not None else f"{self.session_id:#04x}",
      })

  class Response(UDSPacket):
    def __init__(self, data: bytes):
      super().__init__(data)


class ECUResetService(UDSService):
  """
  ECU_RESET (0x11)

  Request Data: \\x11\\xAA

  AA = 0x01 (HARD)
  AA = 0x02 (KEY_OFF_ON)
  AA = 0x03 (SOFT)
  AA = 0x04 (ENABLE_RAPID_POWER_SHUTDOWN)
  AA = 0x05 (DISABLE_RAPID_POWER_SHUTDOWN)
  """

  class Request(UDSPacket):
    def __init__(self, data: bytes):
      assert len(data) == 2
      super().__init__(data)

    @property
    def reset_id(self) -> int:
      return self.data[1]

    @property
    def reset_type(self) -> Optional[RESET_TYPE]:
      return RESET_TYPE(self.reset_id) if self.reset_id in RESET_TYPES else None

    def __str__(self) -> str:
      return self._build_str({
        "reset_type": self.reset_type.name if self.reset_type is not None else f"{self.reset_id:#02x}",
      })

  class Response(UDSPacket):
    def __init__(self, data: bytes):
      super().__init__(data)


class SecurityAccessService(UDSService):
  """
  SECURITY_ACCESS (0x27)
  """

  class Request(UDSPacket):
    def __init__(self, data: bytes):
      assert len(data) == 2
      super().__init__(data)

    @property
    def access_id(self) -> int:
      return self.data[1]

    @property
    def access_type(self) -> Optional[ACCESS_TYPE]:
      return ACCESS_TYPE(self.access_id) if self.access_id in ACCESS_TYPES else None

    def __str__(self) -> str:
      return self._build_str({
        "access_type": self.access_type.name if self.access_type is not None else f"{self.access_id:#02x}",
      })

  class Response(UDSPacket):
    def __init__(self, data: bytes):
      assert len(self.data) > 2
      super().__init__(data)

    @property
    def access_id(self) -> int:
      return self.data[1]

    @property
    def access_type(self) -> Optional[ACCESS_TYPE]:
      return ACCESS_TYPE(self.access_id) if self.access_id in ACCESS_TYPES else None

    @property
    def seed(self) -> bytes:
      return self.data[2:]

    def __str__(self) -> str:
      return self._build_str({
        "access_type": self.access_type.name if self.access_type is not None else f"{self.access_id:#02x}",
        "seed": f"{hexify(self.seed)}",
      })


UDS_SERVICES = {
  SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL: DiagnosticSessionControlService,
  SERVICE_TYPE.ECU_RESET: ECUResetService,
  SERVICE_TYPE.SECURITY_ACCESS: SecurityAccessService,
}


def parse_uds_packet(data: bytes) -> Optional[UDSPacket]:
  service_identifier = int(data[0])
  request = True
  if service_identifier & 0x40 == 0x40:
    request = False
    service_identifier -= 0x40

  service = UDSService
  try:
    service_type = SERVICE_TYPE(service_identifier)
    if service_type in UDS_SERVICES:
      service = UDS_SERVICES[service_type]
  except KeyError:
    pass

  if request:
    return service.Request(data)
  else:
    return service.Response(data)
