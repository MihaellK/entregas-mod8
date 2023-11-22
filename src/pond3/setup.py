from setuptools import find_packages, setup

package_name = 'pond3'

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
    maintainer='mihaell',
    maintainer_email='mihaell.klosowski@gmail.com',
    description='Ponderada 3: ChatBot com Regex',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chatbot = pond3.chatbot:main',
        ],
    },
)
