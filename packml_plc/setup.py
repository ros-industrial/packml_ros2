#!/usr/bin/env python
#
# Software License Agreement
# Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Contributors: Dejanira Araiza Illan, Derrick Ang Ming Yan
#

from setuptools import find_packages
from setuptools import setup

package_name = 'packml_plc'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chen Bainian',
    maintainer_email='chenbn@artc.a-star.edu.sg',
    author='Dejanira Araiza Illan',
    author_email='dejanira.araiza.i@gmail.com',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Users',
        'License :: Apache 2.0',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Packml PLC driver',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'packml_plc_listener = packml_plc.packml_plc_listener:main',
            'packml_plc_sender = packml_plc.packml_plc_sender:main',
        ],
    },
)
