/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <unistd.h>
#include "DriverFramework.hpp"
#include "ERLEBRAIN2_RC_IN.hpp"

using namespace DriverFramework;

class PressureTester
{
public:
	static const int TEST_PASS = 0;
	static const int TEST_FAIL = 1;

    PressureTester()
	{}

	int run(void);

private:
    int         m_pass;
	bool		m_done = false;
};

int PressureTester::run()
{
	DF_LOG_INFO("Entering: run");
	// Default is fail unless pass critera met
	m_pass = TEST_FAIL;

    ErleBrain2RcInput rcin("/dev/mem");

    rcin.init();
    //DF_LOG_INFO("init");
    int ret = rcin.start();

    DF_LOG_INFO("start: %d", ret);

    for(int i=0; i<1000;i++)
        usleep(100000);

    DF_LOG_INFO("Entering: out");

	return m_pass;
}

int do_test();
int do_test()
{
	int ret = Framework::initialize();

	if (ret < 0) {
		return ret;
	}

	PressureTester pt;

	ret = pt.run();

	Framework::shutdown();

	DF_LOG_INFO("Test %s", (ret == PressureTester::TEST_PASS) ? "PASSED" : "FAILED");
	return ret;
}

