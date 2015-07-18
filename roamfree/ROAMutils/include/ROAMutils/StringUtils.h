/*
Copyright (c) 2013-2014 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (cucci@elet.polimi.it)
*/

/*
 * StringUtils.h
 *
 *  Created on: Mar 13, 2014
 *      Author: davide
 */

#ifndef STRINGUTILS_H_
#define STRINGUTILS_H_

#include <string>

namespace ROAMutils {

class StringUtils {

public:

	/**
	 * this formats a timestamp as fixed and mod 100.0
	 */
	static std::string writeNiceTimestamp(double t);
};

} /* namespace ROAMutils */

#endif /* STRINGUTILS_H_ */
