import os

headers = ['hmc5883l.h']
sources = ['hmc5883l.c']

lib = Library('libhmc5883l', sources)

install_prefix = '/usr/local'
install_include = os.path.join(install_prefix, 'include')
install_lib = os.path.join(install_prefix, 'lib')

Alias('install', Install(install_include, headers))
Alias('install', Install(install_lib, lib))
Command('uninstall', None, Delete(FindInstalledFiles()))

Default(lib)
