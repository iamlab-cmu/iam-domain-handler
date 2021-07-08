from distutils.core import setup
  

setup(name='iam_domain_handler',
      version='0.1.0',
      install_requires=[
            'pillar_state',
            'numpy-quaternion',
            'shortuuid'
      ],
      description='Domain Handler Utilities for IAM Lab',
      author='Jacky Liang',
      author_email='jackyliang@cmu.edu',
      packages=['iam_domain_handler']
     )