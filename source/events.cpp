namespace imajuscule::audio {

std::ostream & operator << (std::ostream & os, const onEventResult & e) {
  switch(e) {
    case onEventResult::OK:
      os << "Ok";
      break;
    case onEventResult::DROPPED_NOTE:
      os << "DroppedNote";
      break;
  }
  return os;
}

}
