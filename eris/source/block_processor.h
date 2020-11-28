
#include <functional>
#include <vector>

namespace njones {
template <class SampleType>
class BlockProcessor {
   public:
    BlockProcessor(
        std::function<void(const int, double*, const int)> processor =
            [](const int, double*, const int) -> void {},
        int nchannels = 0,
        int block_size = 0)
        : processor(processor), index(0) {
        resize(nchannels, block_size);
    }

    BlockProcessor(const BlockProcessor& rhs) { this->operator=(rhs); }

    BlockProcessor(BlockProcessor&& rhs) { this->operator=(rhs); }

    ~BlockProcessor() {}

    BlockProcessor& operator=(const BlockProcessor& rhs) {
        processor = rhs.processor;
        nchannels = rhs.nchannels;
        block_size = rhs.block_size;
        index = rhs.index;
        buffer = rhs.buffer;

        return *this;
    }

    BlockProcessor& operator=(BlockProcessor&& rhs) {
        processor = std::move(rhs.processor);
        nchannels = std::move(rhs.nchannels);
        block_size = std::move(rhs.block_size);
        index = std::move(rhs.index);
        buffer = std::move(rhs.buffer);

        return *this;
    }

    void resize(const int nchannels, const int block_size) {
        if (nchannels != this->nchannels) {
            this->nchannels = nchannels;
            buffer = std::vector<std::vector<double>>(nchannels);
            this->block_size = 0;
            index = 0;
        }
        if (this->block_size != block_size) {
            this->block_size = block_size;
            for (int i = 0; i < nchannels; ++i)
                buffer.at(i) = std::vector<double>(block_size);
            index = 0;
        }
    }

    void set_nchannels(const int nchannels) { resize(nchannels, block_size); }

    void set_block_size(const int block_size) { resize(nchannels, block_size); }

    void set_processor(std::function<void(const int, double*, const int)> processor) {
        this->processor = processor;
    }

    int get_nchannels() const { return nchannels; }
    int get_block_size() const { return block_size; }

    void add(SampleType** samples, const int nsamples) {
        int remaining = nsamples;
        int offset = 0;
        int channel_index;
        int channel_remaining;
        int channel_offset;

        while (remaining > 0) {
            for (int channel = 0; channel < nchannels; ++channel) {
                channel_offset = offset;
                channel_index = index;
                channel_remaining = remaining;

                while (channel_remaining-- > 0) {
                    buffer.at(channel).at(channel_index++) = static_cast<double>(samples[channel][channel_offset++]);

                    if (channel_index == block_size) {
                        processor(channel, buffer.at(channel).data(), channel_offset);
                        channel_index = 0;
                        break;
                    }
                }
            }
            remaining = channel_remaining;
            offset = channel_offset;
            index = channel_index;
        }
    }

    void clear() {
        index = 0;
    }

   protected:
    std::function<void(const int, double*, const int)> processor;
    int nchannels;
    int block_size;
    int index;

    std::vector<std::vector<double>> buffer;
};
}  // namespace njones