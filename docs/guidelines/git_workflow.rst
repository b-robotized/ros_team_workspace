==========================
Working as a team with Git
==========================

We all have different experiences and habits working on GIT.
Which are very valid and not to be diminished.
When working together as a group, we should strive towards unifying them and have a common agreement on how we work together.

This is the exact purpose of this document - to set a standard on how development should happen using Git.
At least for our workgroup.

The main branch
===========================

Let's start with the basic setup that every repository should have: a protected ``main``/``master`` branch.
Protecting a branch means that at no point is anyone allowed to push to it directly.
Every modification of the code should happen through pull/merge requests (see guidelines below - Merge/pull requests).

When starting work on a new feature, bugfix, documentation improvement or anything else, follow the ...

Git branch naming convention
============================

| As the saying goes:
| *"A large number of branches with seemingly no purpose on a single repository is a strong sign of upcoming chaos."*
| - Timo

Joking aside, branching a repository should happen for a reason.
And the branch should also exist for a reason.
Despite these two statements being seemingly obvious, experience shows that they are often overlooked.
This can then result in a large number of branches that are unmaintained, forgotten and nobody knows why they exist in the first place.

Therefore, we state that there are two distinctive reasons for a branch to exist: it is actively being worked on or contains code that is useful in a certain scenario.
If the scenario becomes irrelevant, so does the code and the branch that hosts it and should therefore be deleted/archived.

In light of this, we set forth the following guideline.
When creating a branch on a repository, have a clear understanding what are you doing it for.
After this, select a prefix from the list below and finally give it a clear name:

- ``feature/`` or ``feat/`` - When creating a new feature or improving the code
- ``bugfix/`` or ``fix/`` - When fixing a known bug.
- ``hotfix/`` - Those little things you spotted that you wanted to fix (typo in the documentation, missing image, wrong comment, delete commented out line, etc.).
- ``doc/`` - When your task is to extend the documentation.
- ``release/`` - Use this for adding changes to the CHANGELOG.md before a new release.

The name should also be rather descriptive but at the same time not too long. Use dashes ``-`` to separate words.

Merge/pull requests
===================

As discussed above, we will add improvements to our code through pull requests.
These allows the author of the code to present their work to others and request their review.

It is very important that the author of the pull request provide sufficient information to the reviewers!
Therefore, each merge request should contain the following elements:

- A brief description what it does and what changes does it introduce
- Instructions how to test the code and the expected results
- Any additional information that will help the reviewer

.. note::
    **Commenting-out sections of code**

    As a general rule of thumb, if a portion of code is not needed, it should simply not be there.
    When developing locally, we often test things by commenting-out sections of code.
    While this is perfectly fine for when we develop, such commented-out section should not make it into the PR.
    Code is either needed or it is not.

    However, there are of course exceptions to this rule.
    If you think that a commented-out section of code should be left there for whatever reason - note it!
    Simple add a comment at the beginning of this explaining why is this code commented-out and why is it staying: ``# The code blow is commented-out but we decided to keep it because ...``

    This will significantly ease the work of the reviewer when they see such sections.

Releases and versioning
=======================

At one point you will likely find yourself in a situation that you want to label the state of the code with a version.
This is typically done when development milestone is reached.
When using Git, this is done through tags.
One can name a tag whatever they want.
However, we recommend using `Semantic Versioning (SemVer) <https://semver.org/>`_ for naming tags.
In short, this will make your tags (labels) look like this: ``MAJOR.MINOR.PATCH`` (i.e. ``1.2.3``).
If you prefer, you can add the ``v`` prefix: ``v1.2.3``.

One benefit of using tags (or versions) is dependency handling.
Some other components of your project might depend on a specific version of your code.
Using tags allows you to easily point to a specific state of the code.
