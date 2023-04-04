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

#include <swri_console/settings_keys.h>

namespace swri_console
{
  const QString SettingsKeys::DISPLAY_TIMESTAMPS = "Timestamps/DisplayTimestamps";
  const QString SettingsKeys::ABSOLUTE_TIMESTAMPS = "Timestamps/AbsoluteTimestamps";
  const QString SettingsKeys::HUMAN_READABLE_TIME = "Timestamps/HumanReadableTime";
  const QString SettingsKeys::DISPLAY_LOGGER = "Identification/Logger";
  const QString SettingsKeys::DISPLAY_FUNCTION = "Identification/Function";
  const QString SettingsKeys::USE_REGEXPS = "Filters/UseRegexps";
  const QString SettingsKeys::INCLUDE_FILTER = "Filters/IncludeFilter";
  const QString SettingsKeys::EXCLUDE_FILTER = "Filters/ExcludeFilter";
  const QString SettingsKeys::SHOW_DEBUG = "Severity/ShowDebug";
  const QString SettingsKeys::SHOW_INFO = "Severity/ShowInfo";
  const QString SettingsKeys::SHOW_WARN = "Severity/ShowWarn";
  const QString SettingsKeys::SHOW_ERROR = "Severity/ShowError";
  const QString SettingsKeys::SHOW_FATAL = "Severity/ShowFatal";
  const QString SettingsKeys::FOLLOW_NEWEST = "UI/FollowNewest";
  const QString SettingsKeys::FONT = "UI/Font";
  const QString SettingsKeys::DEBUG_COLOR = "Colors/DebugColor";
  const QString SettingsKeys::INFO_COLOR = "Colors/InfoColor";
  const QString SettingsKeys::WARN_COLOR = "Colors/WarnColor";
  const QString SettingsKeys::ERROR_COLOR = "Colors/ErrorColor";
  const QString SettingsKeys::FATAL_COLOR = "Colors/FatalColor";
  const QString SettingsKeys::COLORIZE_LOGS = "Colors/ColorizeLogs";
  const QString SettingsKeys::ALTERNATE_LOG_ROW_COLORS = "Logs/AlternateRowColors";
}
