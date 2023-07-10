/* Copyright (c) 2022, Gonzalo Ferrer
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * test_Sim3.cpp
 *
 *  Created on: 10 July, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include <iostream>
#include <Eigen/LU>
#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp"
#include "mrob/SO3.hpp"
#include "mrob/sim3.hpp"

#include <iostream>

using namespace std;


int main()
{
    // this could be though as the same tests as in SE3, but here we dont have Ln,
    // so it is harder to check.
{
    std::cout << "test constuctor default:\n";
    mrob::Sim3 S;
    S.print();
}

{
    std::cout << "test constuctor direct matrix:\n";
    mrob::Mat4 random = mrob::Mat4::Random();
    mrob::Sim3 S(random);
    S.print();
    std::cout << "regenerate the matrix:\n";
    S.regenerate();
    S.print();
}

{
    std::cout << "test constuctor nu:\n";
    mrob::Mat71 nu;
    nu << 0, 0, 0, 1, 2, 3, 0;
    mrob::Sim3 S(nu);
    S.print();
}

{
    std::cout << "test constuctor rot:\n";
    mrob::Mat71 nu;
    nu << 1.0, 0, 0, 0, 0, 0, 0;
    mrob::Sim3 S(nu);
    S.print();
}

{
    std::cout << "test constuctor rot:\n";
    mrob::Mat71 nu;
    nu << 0, 1.0, 0, 0, 0, 0, 0;
    mrob::Sim3 S(nu);
    S.print();
}

{
    std::cout << "test constuctor rot:\n";
    mrob::Mat71 nu;
    nu << 0.3, 1.0, -0.2, 0, 0, 0, 0;
    mrob::Sim3 S(nu);
    S.print();
}

{
    std::cout << "test constuctor tranlation scale:\n";
    mrob::Mat71 nu;
    nu << 0, 0, 0, 1, 2, 3, -0.1;
    mrob::Sim3 S(nu);
    S.print();
}

{
    std::cout << "test constuctor tranlation scale:\n";
    mrob::Mat71 nu;
    nu << 0.3, 1.0, -0.2, 0, 0, 0, 0.1;
    mrob::Sim3 S(nu);
    S.print();
}

{
    std::cout << "test constuctor tranlation scale:\n";
    mrob::Mat71 nu;
    nu << 0.3, 1.0, -0.2, 1, 2, 3, 1e-5;
    mrob::Sim3 S(nu);
    S.print();
}

    return 0;
}