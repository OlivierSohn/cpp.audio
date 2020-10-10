namespace imajuscule::audio {

namespace {

std::optional<Note> parse(Note note, std::string const & str, size_t pos) {
  if(str.size() <= pos) {
    return note;
  }
  if(str.size() > pos + 1) {
    LG(ERR, "Note error: '%s' has too many characters", str.c_str());
    return {};
  }
  switch(str[pos]) {
    case 'd':
    case 'D':
    case '#':
    case 's':
      if(note == Note::Si) {
        return Note::Do;
      }
      return safe_cast<Note>(to_underlying(note) + 1);
    case 'b':
    case 'B':
    case 'f':
      if(note == Note::Do) {
        return Note::Si;
      }
      return safe_cast<Note>(to_underlying(note) - 1);
  }
  LG(ERR, "Note error: '%s' has an unrecognized end character", str.c_str());
  return {};
}

struct NoteClass {
  NoteClass(const char * str, Note n) : str(str), enum_(n) {}
  const char * str;
  Note enum_;
};

std::array<NoteClass,7> const notes = {{
  {"do", Note::Do},
  {"re", Note::Re},
  {"mi", Note::Mi},
  {"fa", Note::Fa},
  {"sol", Note::Sol},
  {"la", Note::La},
  {"si", Note::Si},
}};

std::optional<Note> parseNote(std::string const & str) {
  for(auto const & note : notes) {
    if(auto pos = ibegins_with(str, note.str)) {
      return parse( note.enum_, str, pos);
    }
  }
  LG(ERR, "Note error: '%s' is not recognized", str.c_str());
  return {};
}

struct NoteAlgo {
  
  bool run(std::string const & score) {
    resetCurrent();
    
    notespecs.reserve(score.size());
    
    size_t pos = 0;
    while(pos < score.size()) {
      auto next = score.find_first_of(" .-", pos);
      if(next == pos) {
        switch(score[pos]) {
          case ' ':
            break;
          case '.':
            if(current.note) {
              make_note();
            }
            ++ current.duration;
            break;
          case '-':
            ++ current.duration;
            break;
          default:
            LG(ERR, "unsupported Note postfix '%c'", score[pos]);
            Assert(0);
            return false;
        }
        ++pos;
      }
      else {
        make_pending();
        auto str = score.substr(pos, next-pos);
        pos = next;
        if(str.empty()) {
          return false;
        }
        Assert(current.duration == 0);
        current.loud = isupper(str[0]);
        std::optional<Note> const n = parseNote(std::move(str));
        if(!n) {
          return false;
        }
        current.note = *n;
        current.duration = 1;
        // do not include the note yet, we need to know how long it will last
      }
    }
    
    make_pending();
    return true;
  }
  
  StackVector<NoteSpec> notespecs;
private:
  NoteSpec current;
  
  void make_pending() {
    if(current.duration == 0) {
      return;
    }
    if(!current.note) {
      make_silence();
    } else {
      make_note();
    }
  }
  
  void make_note() {
    Assert(current.note);
    notespecs.emplace_back(current);
    resetCurrent();
  }
  
  void make_silence() {
    Assert(!current.note);
    notespecs.emplace_back(current);
    resetCurrent();
  }
  
  void resetCurrent()
  {
    current.note.reset();
    current.duration = 0;
    current.loud = false;
  }
};

void normalize(std::string & score) {
  for(auto &c : score) {
    // allow user to write ré or re or RÉ
    if(c == safe_cast<char>(130) /*'é'*/ || c == safe_cast<char>(144) /*'É'*/) {
      c = 'e';
    }
  }
}
}

StackVector<NoteSpec> parseMusic(std::string score) {
  normalize(score);
  NoteAlgo a;
  if( !a.run(std::move(score))) {
    LG(ERR, "not all music could be parsed");
  }
  return std::move(a.notespecs);
}

}
