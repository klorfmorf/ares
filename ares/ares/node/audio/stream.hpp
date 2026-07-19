struct Stream : Audio {
  DeclareClass(Stream, "audio.stream")
  using Audio::Audio;

  auto channels() const -> u32 { return _channels.size(); }
  auto frequency() const -> f64 { return _frequency; }
  auto resamplerFrequency() const -> f64 { return _resamplerFrequency; }
  auto muted() const -> bool { return _muted; }

  auto setChannels(u32 channels) -> void;
  auto setFrequency(f64 frequency) -> void;
  auto setResamplerFrequency(f64 resamplerFrequency) -> void;
  auto setMuted(bool muted) -> void;

  auto resetFilters() -> void;
  auto addLowPassFilter(f64 cutoffFrequency, u32 order, u32 passes = 1) -> void;
  auto addHighPassFilter(f64 cutoffFrequency, u32 order, u32 passes = 1) -> void;
  auto addLowShelfFilter(f64 cutoffFrequency, u32 order, f64 gain, f64 slope) -> void;
  auto addHighShelfFilter(f64 cutoffFrequency, u32 order, f64 gain, f64 slope) -> void;

  auto pending() const -> bool;
  auto read(f64 samples[]) -> u32;
  auto write(const f64 samples[]) -> void;

  template<typename... P>
  auto frame(P&&... p) -> void {
    if(runAhead()) return;
    f64 samples[sizeof...(p)] = {f64(std::forward<P>(p))...};
    write(samples);
  }

protected:
  struct Resampler;
  struct ResamplerDeleter {
    auto operator()(Resampler* resampler) const -> void;
  };

  struct Filter {
    enum class Mode : u32 { OnePole, Biquad } mode;
    enum class Type : u32 { None, LowPass, HighPass, LowShelf, HighShelf } type;
    enum class Order : u32 { None, First, Second } order;
    DSP::IIR::OnePole onePole;
    DSP::IIR::Biquad biquad;
  };
  struct Channel {
    auto write(f64 sample) -> void;

    std::vector<Filter> filters;
    std::vector<f64> input;
    queue<f64> output;
    std::unique_ptr<Resampler, ResamplerDeleter> resampler;
  };
  std::vector<Channel> _channels;
  f64 _frequency = 48000.0;
  f64 _resamplerFrequency = 48000.0;
  u32 _resamplerBlockSize = 0;
  u32 _resamplerInputSize = 0;
  bool _muted = false;
};
