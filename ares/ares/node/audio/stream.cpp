struct Stream::Resampler {
  Resampler(f64 inputFrequency, f64 outputFrequency, u32 maxInputSize)
  : instance(inputFrequency, outputFrequency, maxInputSize) {
  }

  auto maxOutputSize(u32 inputSize) const -> u32 {
    return instance.getMaxOutLen(inputSize);
  }

  auto process(f64 input[], u32 inputSize, f64*& output) -> u32 {
    return instance.process(input, inputSize, output);
  }

private:
  r8b::CDSPResampler16 instance;
};

auto Stream::ResamplerDeleter::operator()(Resampler* resampler) const -> void {
  delete resampler;
}

auto Stream::Channel::write(f64 sample) -> void {
  if(output.full()) output.read();
  output.write(sample);
}

auto Stream::setChannels(u32 channels) -> void {
  _channels.clear();
  _channels.resize(channels);
  setResamplerFrequency(_resamplerFrequency);
}

auto Stream::setFrequency(f64 frequency) -> void {
  _frequency = frequency;
  setResamplerFrequency(_resamplerFrequency);
}

auto Stream::setResamplerFrequency(f64 resamplerFrequency) -> void {
  _resamplerFrequency = resamplerFrequency;
  _resamplerBlockSize = std::clamp((u32)ceil(_frequency / 1000.0), 16u, 4096u);
  _resamplerInputSize = 0;

  for(auto& channel : _channels) {
    channel.input.resize(_resamplerBlockSize);
    channel.resampler.reset();

    u32 outputQueueSize = max(1u, (u32)ceil(_resamplerFrequency * 0.05));
    if(_frequency != _resamplerFrequency) {
      channel.resampler.reset(new Resampler(
        _frequency, _resamplerFrequency, _resamplerBlockSize
      ));
      outputQueueSize = max(outputQueueSize, channel.resampler->maxOutputSize(_resamplerBlockSize) * 2);
    }
    channel.output.resize(outputQueueSize);
  }
}

auto Stream::setMuted(bool muted) -> void {
  _muted = muted;
}

auto Stream::resetFilters() -> void {
  for(auto& channel : _channels) {
    channel.filters.clear();
  }
}

auto Stream::addLowPassFilter(f64 cutoffFrequency, u32 order, u32 passes) -> void {
  for(auto& channel : _channels) {
    for(u32 pass : range(passes)) {
      if(order == 1) {
        Filter filter{Filter::Mode::OnePole, Filter::Type::LowPass, Filter::Order::First};
        filter.onePole.reset(DSP::IIR::OnePole::Type::LowPass, cutoffFrequency, _frequency);
        channel.filters.push_back(filter);
      }
      if(order == 2) {
        Filter filter{Filter::Mode::Biquad, Filter::Type::LowPass, Filter::Order::Second};
        f64 q = DSP::IIR::Biquad::butterworth(passes * 2, pass);
        filter.biquad.reset(DSP::IIR::Biquad::Type::LowPass, cutoffFrequency, _frequency, q);
        channel.filters.push_back(filter);
      }
    }
  }
}

auto Stream::addHighPassFilter(f64 cutoffFrequency, u32 order, u32 passes) -> void {
  for(auto& channel : _channels) {
    for(u32 pass : range(passes)) {
      if(order == 1) {
        Filter filter{Filter::Mode::OnePole, Filter::Type::HighPass, Filter::Order::First};
        filter.onePole.reset(DSP::IIR::OnePole::Type::HighPass, cutoffFrequency, _frequency);
        channel.filters.push_back(filter);
      }
      if(order == 2) {
        Filter filter{Filter::Mode::Biquad, Filter::Type::HighPass, Filter::Order::Second};
        f64 q = DSP::IIR::Biquad::butterworth(passes * 2, pass);
        filter.biquad.reset(DSP::IIR::Biquad::Type::HighPass, cutoffFrequency, _frequency, q);
        channel.filters.push_back(filter);
      }
    }
  }
}

auto Stream::addLowShelfFilter(f64 cutoffFrequency, u32 order, f64 gain, f64 slope) -> void {
  for(auto& channel : _channels) {
    if(order == 2) {
      Filter filter{Filter::Mode::Biquad, Filter::Type::LowShelf, Filter::Order::Second};
      f64 q = DSP::IIR::Biquad::shelf(gain, slope);
      filter.biquad.reset(DSP::IIR::Biquad::Type::LowShelf, cutoffFrequency, _frequency, q);
      channel.filters.push_back(filter);
    }
  }
}

auto Stream::addHighShelfFilter(f64 cutoffFrequency, u32 order, f64 gain, f64 slope) -> void {
  for(auto& channel : _channels) {
    if(order == 2) {
      Filter filter{Filter::Mode::Biquad, Filter::Type::HighShelf, Filter::Order::Second};
      f64 q = DSP::IIR::Biquad::shelf(gain, slope);
      filter.biquad.reset(DSP::IIR::Biquad::Type::HighShelf, cutoffFrequency, _frequency, q);
      channel.filters.push_back(filter);
    }
  }
}

auto Stream::pending() const -> bool {
  return !_channels.empty() && _channels[0].output.pending();
}

auto Stream::read(f64 samples[]) -> u32 {
  for(u32 c : range(_channels.size())) {
    samples[c] = _channels[c].output.read() * !muted();
  }
  return _channels.size();
}

auto Stream::write(const f64 samples[]) -> void {
  bool resampling = _frequency != _resamplerFrequency;

  for(u32 c : range(_channels.size())) {
    f64 sample = samples[c] + 1e-25;  //constant offset used to suppress denormals
    for(auto& filter : _channels[c].filters) {
      switch(filter.mode) {
      case Filter::Mode::OnePole: sample = filter.onePole.process(sample); break;
      case Filter::Mode::Biquad: sample = filter.biquad.process(sample); break;
      }
    }
    if(resampling) _channels[c].input[_resamplerInputSize] = sample;
    else _channels[c].write(sample);
  }

  if(resampling && ++_resamplerInputSize == _resamplerBlockSize) {
    u32 outputCount = 0;
    for(u32 c : range(_channels.size())) {
      f64* output = nullptr;
      u32 count = _channels[c].resampler->process(
        _channels[c].input.data(), _resamplerInputSize, output
      );
      if(c == 0) outputCount = count;
      else assert(count == outputCount);
      for(u32 n : range(count)) _channels[c].write(output[n]);
    }
    _resamplerInputSize = 0;
  }

  //if there are samples pending, then alert the frontend to possibly process them.
  //this will generally happen when every audio stream has pending samples to be mixed.
  if(pending()) platform->audio(std::static_pointer_cast<Core::Audio::Stream>(shared_from_this()));
}
