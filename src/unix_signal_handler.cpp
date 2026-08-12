// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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

#include <swri_console/unix_signal_handler.h>

#include <QCoreApplication>
#include <QSocketNotifier>

#include <sys/socket.h>
#include <unistd.h>

#include <csignal>

namespace swri_console
{
int UnixSignalHandler::sigintFd[2];
int UnixSignalHandler::sigtermFd[2];

UnixSignalHandler::UnixSignalHandler(QObject* parent) :
  QObject(parent),
  sn_int_(nullptr),
  sn_term_(nullptr)
{
}

void UnixSignalHandler::setup()
{
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sigintFd)) {
    qFatal("UnixSignalHandler: couldn't create SIGINT socketpair");
  }
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sigtermFd)) {
    qFatal("UnixSignalHandler: couldn't create SIGTERM socketpair");
  }

  sn_int_ = new QSocketNotifier(sigintFd[0], QSocketNotifier::Read, this);
  connect(sn_int_, SIGNAL(activated(int)), this, SLOT(handleSigInt()));

  sn_term_ = new QSocketNotifier(sigtermFd[0], QSocketNotifier::Read, this);
  connect(sn_term_, SIGNAL(activated(int)), this, SLOT(handleSigTerm()));

  struct sigaction int_action;
  int_action.sa_handler = UnixSignalHandler::intSignalHandler;
  ::sigemptyset(&int_action.sa_mask);
  int_action.sa_flags = SA_RESTART;
  if (::sigaction(SIGINT, &int_action, nullptr)) {
    qFatal("UnixSignalHandler: couldn't install SIGINT handler");
  }

  struct sigaction term_action;
  term_action.sa_handler = UnixSignalHandler::termSignalHandler;
  ::sigemptyset(&term_action.sa_mask);
  term_action.sa_flags = SA_RESTART;
  if (::sigaction(SIGTERM, &term_action, nullptr)) {
    qFatal("UnixSignalHandler: couldn't install SIGTERM handler");
  }
}

void UnixSignalHandler::intSignalHandler(int)
{
  char a = 1;
  ssize_t unused = ::write(sigintFd[1], &a, sizeof(a));
  (void)unused;
}

void UnixSignalHandler::termSignalHandler(int)
{
  char a = 1;
  ssize_t unused = ::write(sigtermFd[1], &a, sizeof(a));
  (void)unused;
}

void UnixSignalHandler::handleSigInt()
{
  sn_int_->setEnabled(false);
  char tmp;
  ssize_t unused = ::read(sigintFd[0], &tmp, sizeof(tmp));
  (void)unused;

  QCoreApplication::quit();

  sn_int_->setEnabled(true);
}

void UnixSignalHandler::handleSigTerm()
{
  sn_term_->setEnabled(false);
  char tmp;
  ssize_t unused = ::read(sigtermFd[0], &tmp, sizeof(tmp));
  (void)unused;

  QCoreApplication::quit();

  sn_term_->setEnabled(true);
}
}
