/*
 * Copyright (C) 2015 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2016 Furrtek
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ui_search.hpp"

#include "baseband_api.hpp"
#include "binder.hpp"
#include "string_format.hpp"
#include "ui_freqman.hpp"
#include "freqman_db.hpp"
#include "audio.hpp"

#include <algorithm>

using namespace portapack;
namespace pmem = portapack::persistent_memory;

namespace ui {

void SearchLogger::log_data(SearchRecentEntry& data) {
    log_file.write_entry(";" + to_string_short_freq(data.frequency) + ";" + std::to_string(data.duration));
}

template <>
void RecentEntriesTable<SearchRecentEntries>::draw(
    const Entry& entry,
    const Rect& target_rect,
    Painter& painter,
    const Style& style,
    RecentEntriesColumns& columns) {
    std::string str_duration = "";

    if (entry.duration < 600)
        str_duration = to_string_dec_uint(entry.duration / 10) + "." + to_string_dec_uint(entry.duration % 10) + "s";
    else
        str_duration = to_string_dec_uint(entry.duration / 600) + "m" + to_string_dec_uint((entry.duration / 10) % 60) + "s";

    str_duration.resize(11, ' ');
    std::string freq = to_string_short_freq(entry.frequency);
    freq.resize(columns.at(0).second, ' ');
    painter.draw_string(target_rect.location(), style, freq + " " + entry.time + " " + str_duration);
}

/* SearchView ********************************************/

SearchView::SearchView(
    NavigationView& nav)
    : nav_(nav) {
    spectrum_row.resize(240);
    baseband::run_image(portapack::spi_flash::image_tag_wideband_spectrum);

    if (!gradient.load_file(default_gradient_file)) {
        gradient.set_default();
    }

    add_children({&labels,
                  &field_frequency_min,
                  &field_frequency_max,
                  &field_lna,
                  &field_vga,
                  &field_threshold,
                  &text_mean,
                  &text_slices,
                  &text_rate,
                  &text_infos,
                  &vu_max,
                  &progress_timers,
                  &check_snap,
                  &options_snap,
                  &big_display,
                  &check_listen,
                  &button_ignore,
                  &check_log,
                  &recent_entries_view});

    baseband::set_spectrum(SEARCH_SLICE_WIDTH, 31);

    recent_entries_view.set_parent_rect({0, 29 * 8, screen_width, screen_height - 29 * 8});
    recent_entries_view.on_select = [this, &nav](const SearchRecentEntry& entry) {
        nav.push<FrequencySaveView>(entry.frequency);
    };

    text_mean.set_style(Theme::getInstance()->fg_medium);
    text_slices.set_style(Theme::getInstance()->fg_medium);
    text_rate.set_style(Theme::getInstance()->fg_medium);
    progress_timers.set_style(Theme::getInstance()->fg_medium);
    big_display.set_style(Theme::getInstance()->fg_medium);

    field_frequency_min.set_step(100'000);
    bind(field_frequency_min, settings_.freq_min, nav, [this](auto) {
        on_range_changed();
    });

    field_frequency_max.set_step(100'000);
    bind(field_frequency_max, settings_.freq_max, nav, [this](auto) {
        on_range_changed();
    });

    check_log.on_select = [this](Checkbox&, bool v) {
        logging = v;
        if (logging) {
            logger.append(logs_dir.string() + "/SEARCH_" + to_string_timestamp(rtc_time::now()) + ".CSV");
            logger.write_header();
        }
    };

    bind(field_threshold, settings_.power_threshold);
    bind(check_snap, settings_.snap_search);
    bind(options_snap, settings_.snap_step);
    bind(check_listen, settings_.auto_listen);

    button_ignore.on_select = [this](Button&) {
        auto sel = recent_entries_view.selected();
        if (!sel) return;

        auto freq = sel->frequency;
        add_to_ignore_list(freq);

        auto it = find(recent, freq);
        if (it != recent.end())
            recent.erase(it);
        recent_entries_view.set_dirty();

        if (locked && resolved_frequency == freq) {
            if (listening) stop_listening();
            locked = false;
            locked_ignored = false;
            detect_timer = 0;
            release_timer = 0;
            listen_timer = 0;
            text_infos.set("Listening");
            big_display.set_style(Theme::getInstance()->fg_medium);
        }
    };

    progress_timers.set_max(DETECT_DELAY);

    load_ignore_list();
    on_range_changed();
    receiver_model.enable();

    if (pmem::beep_on_packets()) {
        audio::set_rate(audio::Rate::Hz_24000);
        audio::output::start();
    }
}

SearchView::~SearchView() {
    audio::output::stop();
    receiver_model.disable();
    baseband::shutdown();
}

void SearchView::on_show() {
    baseband::spectrum_streaming_start();
}

void SearchView::on_hide() {
    if (listening)
        stop_listening();
    baseband::spectrum_streaming_stop();
}

void SearchView::focus() {
    field_frequency_min.focus();
}

void SearchView::do_detection() {
    uint8_t power_max = 0;
    int32_t bin_max = -1;
    uint32_t slice_max = 0;
    uint32_t snap_value;
    uint8_t power;
    rtc::RTC datetime;
    std::string str_approx, str_timestamp;

    // Display spectrum
    bin_skip_acc = 0;
    pixel_index = 0;
    uint16_t center_align_start = (screen_width - spectrum_row.size()) / 2;
    display.draw_pixels({{center_align_start, 88}, {(Dim)spectrum_row.size(), 1}}, spectrum_row);

    mean_power = mean_acc / (SEARCH_BIN_NB_NO_DC * slices_nb);
    mean_acc = 0;

    overall_power_max = 0;

    // Find max power over threshold for all slices
    for (size_t slice = 0; slice < slices_nb; slice++) {
        power = slices[slice].max_power;
        if (power > overall_power_max)
            overall_power_max = power;

        if ((power >= mean_power + settings_.power_threshold) && (power > power_max)) {
            power_max = power;
            bin_max = slices[slice].max_index;
            slice_max = slice;
        }
    }

    // Lock / release
    if ((bin_max >= last_bin - 2) && (bin_max <= last_bin + 2) && (bin_max > -1) && (slice_max == last_slice)) {
        // Staying around the same bin
        if (detect_timer >= DETECT_DELAY) {
            if ((bin_max != locked_bin) || (!locked)) {
                if (!locked) {
                    resolved_frequency = slices[slice_max].center_frequency + (SEARCH_BIN_WIDTH * (bin_max - 128));

                    // Sub-bin refinement: parabolic interpolation using the power of the
                    // neighboring bins narrows the ~9.8kHz FFT bin resolution down to a
                    // fraction of a bin for a clean, single peak.
                    {
                        int32_t bin_left = slices[slice_max].power_left;
                        int32_t bin_center = slices[slice_max].max_power;
                        int32_t bin_right = slices[slice_max].power_right;
                        int32_t denom = bin_left - 2 * bin_center + bin_right;
                        if (bin_left > 0 && bin_right > 0 && denom < 0) {
                            float bin_offset = 0.5f * (bin_left - bin_right) / (float)denom;
                            if (bin_offset > 0.5f) bin_offset = 0.5f;
                            if (bin_offset < -0.5f) bin_offset = -0.5f;
                            resolved_frequency += (int64_t)(bin_offset * SEARCH_BIN_WIDTH);
                        }
                    }

                    if (check_snap.value()) {
                        snap_value = options_snap.selected_index_value();
                        resolved_frequency = round(resolved_frequency / snap_value) * snap_value;
                    }

                    locked_ignored = is_ignored(resolved_frequency);

                    if (locked_ignored) {
                        text_infos.set("Ignored");
                        big_display.set_style(Theme::getInstance()->fg_medium);
                        locked = true;
                        locked_bin = bin_max;
                    } else if ((resolved_frequency >= settings_.freq_min) && (resolved_frequency <= settings_.freq_max)) {
                        // Check range
                        duration = 0;

                        auto& entry = ::on_packet(recent, resolved_frequency);

                        rtcGetTime(&RTCD1, &datetime);
                        str_timestamp = to_string_dec_uint(datetime.hour(), 2, '0') + ":" +
                                        to_string_dec_uint(datetime.minute(), 2, '0') + ":" +
                                        to_string_dec_uint(datetime.second(), 2, '0');
                        entry.set_time(str_timestamp);
                        recent_entries_view.set_dirty();

                        text_infos.set("Locked ! ");
                        big_display.set_style(Theme::getInstance()->fg_green);

                        locked = true;
                        locked_bin = bin_max;
                        if (pmem::beep_on_packets()) {
                            baseband::request_audio_beep(1000, 24000, 60);
                        }

                        if (settings_.auto_listen)
                            start_listening(resolved_frequency);
                    } else
                        text_infos.set("Out of range");
                }

                big_display.set(resolved_frequency);
            }
        }
        release_timer = 0;
    } else {
        detect_timer = 0;
        if (locked) {
            if (release_timer >= RELEASE_DELAY) {
                locked = false;

                if (!locked_ignored) {
                    auto& entry = ::on_packet(recent, resolved_frequency);
                    entry.set_duration(duration);
                    if (logging) logger.log_data(entry);
                    recent_entries_view.set_dirty();
                }
                locked_ignored = false;

                text_infos.set("Listening");
                big_display.set_style(Theme::getInstance()->fg_medium);
            }
        }
    }

    last_bin = bin_max;
    last_slice = slice_max;
    search_counter++;

    // Refresh red tick
    portapack::display.fill_rectangle({last_tick_pos, 90, 1, 6}, Theme::getInstance()->fg_red->background);
    if (bin_max > -1) {
        last_tick_pos = (Coord)(bin_max / slices_nb) + center_align_start;
        portapack::display.fill_rectangle({last_tick_pos, 90, 1, 6}, Theme::getInstance()->fg_red->foreground);
    }
}

void SearchView::do_timers() {
    if (timing_div >= 60) {
        // ~1Hz

        timing_div = 0;

        // Update scan rate
        text_rate.set(to_string_dec_uint(search_counter, 3));
        search_counter = 0;
    }

    if (timing_div % 12 == 0) {
        // ~5Hz

        // Update power levels
        text_mean.set(to_string_dec_uint(mean_power, 3));

        vu_max.set_value(overall_power_max);
        vu_max.set_mark(mean_power + settings_.power_threshold);
    }

    if (timing_div % 6 == 0) {
        // ~10Hz

        // Update timing indicator
        if (locked) {
            progress_timers.set_max(RELEASE_DELAY);
            progress_timers.set_value(RELEASE_DELAY - release_timer);
        } else {
            progress_timers.set_max(DETECT_DELAY);
            progress_timers.set_value(detect_timer);
        }

        // Increment timers
        if (detect_timer < DETECT_DELAY) detect_timer++;
        if (release_timer < RELEASE_DELAY) release_timer++;

        if (locked) duration++;

        if (listening) {
            // Safety cap only; silence-based release is handled by on_listen_statistics().
            if (++listen_timer >= AUTO_LISTEN_MAX_TICKS)
                finish_listening();
        }
    }

    timing_div++;
}

void SearchView::on_channel_spectrum(const ChannelSpectrum& spectrum) {
    // Sweep is paused while auto-listen has switched the baseband to NFM audio.
    if (listening) return;

    uint8_t max_power = 0;
    int16_t max_bin = 0;
    uint8_t power;
    size_t bin;

    baseband::spectrum_streaming_stop();

    // Center 12 bins are ignored (DC spike is blanked)
    // Leftmost and rightmost 2 bins are ignored
    auto bin_power = [&spectrum](int32_t b) -> uint8_t {
        if ((b < 2) || (b > 253) || ((b >= 122) && (b < 134)))
            return 0;
        return (b < 128) ? spectrum.db[128 + b] : spectrum.db[b - 128];
    };

    // Add pixels to spectrum display and find max power for this slice
    for (bin = 0; bin < 256; bin++) {
        power = bin_power(bin);

        add_spectrum_pixel(gradient.lut[power]);

        mean_acc += power;
        if (power > max_power) {
            max_power = power;
            max_bin = bin;
        }
    }

    slices[slice_counter].max_power = max_power;
    slices[slice_counter].max_index = max_bin;
    // Neighboring bin power around the peak, for sub-bin interpolation in do_detection().
    slices[slice_counter].power_left = bin_power(max_bin - 1);
    slices[slice_counter].power_right = bin_power(max_bin + 1);

    if (slices_nb > 1) {
        // Slice sequence
        if (slice_counter >= slices_nb) {
            slice_counter = 0;
            do_detection();
        } else
            slice_counter++;

        // do_detection() may have switched the baseband to NFM audio (auto-listen);
        // the sweep is paused until stop_listening()/finish_listening() restores it.
        if (listening) return;

        receiver_model.set_target_frequency(slices[slice_counter].center_frequency);
        baseband::set_spectrum(SEARCH_SLICE_WIDTH, 31);  // Clear
    } else {
        // Unique slice
        do_detection();

        if (listening) return;
    }

    baseband::spectrum_streaming_start();
}

void SearchView::on_range_changed() {
    rf::Frequency slices_span;
    rf::Frequency center_frequency;
    int64_t offset;
    size_t slice;

    // TODO: enforce min < max?
    search_span = abs(settings_.freq_max - settings_.freq_min);

    if (search_span > SEARCH_SLICE_WIDTH) {
        // ex: 100M~115M (15M span):
        // slices_nb = (115M-100M)/2.5M = 6
        slices_nb = (search_span + SEARCH_SLICE_WIDTH - 1) / SEARCH_SLICE_WIDTH;
        if (slices_nb > 32) {
            text_slices.set("!!");
            slices_nb = 32;
        } else {
            text_slices.set(to_string_dec_uint(slices_nb, 2, ' '));
        }
        // slices_span = 6 * 2.5M = 15M
        slices_span = slices_nb * SEARCH_SLICE_WIDTH;
        // offset = 0 + 2.5/2 = 1.25M
        offset = ((search_span - slices_span) / 2) + (SEARCH_SLICE_WIDTH / 2);
        // slice_start = 100M + 1.25M = 101.25M
        center_frequency = std::min(settings_.freq_min, settings_.freq_max) + offset;

        for (slice = 0; slice < slices_nb; slice++) {
            slices[slice].center_frequency = center_frequency;
            center_frequency += SEARCH_SLICE_WIDTH;
        }
    } else {
        slices[0].center_frequency = (settings_.freq_max + settings_.freq_min) / 2;
        receiver_model.set_target_frequency(slices[0].center_frequency);

        slices_nb = 1;
        text_slices.set(" 1");
    }

    bin_skip_frac = 0xF000 / slices_nb;

    slice_counter = 0;
}

// Freeze the sweep and switch the baseband to NFM audio on the locked frequency.
void SearchView::start_listening(rf::Frequency freq) {
    baseband::spectrum_streaming_stop();
    receiver_model.disable();
    baseband::shutdown();

    baseband::run_image(portapack::spi_flash::image_tag_nfm_audio);
    receiver_model.set_sampling_rate(3'072'000);
    receiver_model.set_baseband_bandwidth(1'750'000);
    receiver_model.set_modulation(ReceiverModel::Mode::NarrowbandFMAudio);
    receiver_model.set_nbfm_configuration(0);
    receiver_model.set_target_frequency(freq);
    receiver_model.enable();

    audio::output::start();

    listening = true;
    listen_timer = 0;
    listen_hang_timer = 0;
    text_infos.set("ON AIR (audio)");
}

// Switch the baseband back to the wideband spectrum sweep and resume scanning.
void SearchView::stop_listening() {
    if (!listening) return;

    audio::output::stop();
    receiver_model.disable();
    baseband::shutdown();

    baseband::run_image(portapack::spi_flash::image_tag_wideband_spectrum);
    receiver_model.set_sampling_rate(SEARCH_SLICE_WIDTH);
    receiver_model.set_baseband_bandwidth(SEARCH_SLICE_WIDTH / 2);
    receiver_model.set_modulation(ReceiverModel::Mode::SpectrumAnalysis);
    receiver_model.set_target_frequency(slices[slice_counter].center_frequency);
    receiver_model.enable();

    baseband::set_spectrum(SEARCH_SLICE_WIDTH, 31);
    baseband::spectrum_streaming_start();

    listening = false;
}

// Called when auto-listen releases (silence timeout or safety cap): finalize the recent entry and resume sweeping.
void SearchView::finish_listening() {
    stop_listening();

    if (!locked_ignored) {
        auto& entry = ::on_packet(recent, resolved_frequency);
        entry.set_duration(duration);
        if (logging) logger.log_data(entry);
        recent_entries_view.set_dirty();
    }
    locked_ignored = false;

    locked = false;
    detect_timer = 0;
    release_timer = 0;
    listen_timer = 0;
    listen_hang_timer = 0;
    text_infos.set("Listening");
    big_display.set_style(Theme::getInstance()->fg_medium);
}

// Hold the auto-listen while RSSI stays above squelch; release after a period of silence.
void SearchView::on_listen_statistics(const ChannelStatistics& statistics) {
    if (!listening) return;
    if (listen_timer < AUTO_LISTEN_WARMUP_TICKS) return;  // Let AGC/filters settle first

    if (statistics.max_db > AUTO_LISTEN_SQUELCH_DB) {
        listen_hang_timer = 0;
    } else if (++listen_hang_timer >= AUTO_LISTEN_HANG_TICKS) {
        finish_listening();
    }
}

bool SearchView::is_ignored(rf::Frequency freq) const {
    return std::find(ignored_frequencies.begin(), ignored_frequencies.end(), freq) != ignored_frequencies.end();
}

void SearchView::load_ignore_list() {
    ignored_frequencies.clear();

    FreqmanDB db;
    if (!db.open(get_freqman_path(ignore_freqman_file)))
        return;

    for (auto entry : db) {
        if (entry.type == freqman_type::Single && entry.frequency_a > 0)
            ignored_frequencies.push_back(entry.frequency_a);
    }
}

void SearchView::add_to_ignore_list(rf::Frequency freq) {
    if (is_ignored(freq))
        return;

    FreqmanDB db;
    if (db.open(get_freqman_path(ignore_freqman_file), /*create*/ true)) {
        freqman_entry entry{
            .frequency_a = freq,
            .type = freqman_type::Single,
        };
        db.append_entry(entry);
    }

    ignored_frequencies.push_back(freq);
}

void SearchView::add_spectrum_pixel(Color color) {
    // Is avoiding floats really necessary?
    bin_skip_acc += bin_skip_frac;
    if (bin_skip_acc < 0x10000)
        return;

    bin_skip_acc -= 0x10000;

    if (pixel_index < spectrum_row.size())
        spectrum_row[pixel_index++] = color;
}

} /* namespace ui */
