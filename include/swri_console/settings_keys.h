// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#ifndef SWRI_CONSOLE_SETTINGS_KEYS_H
#define SWRI_CONSOLE_SETTINGS_KEYS_H

#include <QString>

namespace swri_console
{
  /**
   * This is simply a convenient place for storing all of the keys used for
   * saving settings files.  Always referring to the consts here means we don't
   * have to worry about problems due to typos, and changing a setting name
   * in the future will be easy.
   */
  class SettingsKeys
  {
  public:
    static const QString DISPLAY_TIMESTAMPS;
    static const QString ABSOLUTE_TIMESTAMPS;
    static const QString USE_REGEXPS;
    static const QString INCLUDE_FILTER;
    static const QString EXCLUDE_FILTER;
    static const QString SHOW_DEBUG;
    static const QString SHOW_INFO;
    static const QString SHOW_WARN;
    static const QString SHOW_ERROR;
    static const QString SHOW_FATAL;
    static const QString FOLLOW_NEWEST;
    static const QString FONT;
    static const QString DEBUG_COLOR;
    static const QString INFO_COLOR;
    static const QString WARN_COLOR;
    static const QString ERROR_COLOR;
    static const QString FATAL_COLOR;
    static const QString COLORIZE_LOGS;
    static const QString ALTERNATE_LOG_ROW_COLORS;
  };
}


#endif //SWRI_CONSOLE_SETTINGS_KEYS_H
