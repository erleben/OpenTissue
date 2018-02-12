__TOC__

==OpenTissue Code Review==

It has been decided, that all code which should be part of either the
development-branch, or the release-branch should be subjected to a
code review in order to make sure that the code conforms to
OpenTissue, is well documented, and to find bugs.
There are two different reviews, depending on whether the code is
being submitted to the development-branch, or the release-branch.

For code submitted to the development-branch, a light review should be
performed, and for code submitted to the release-branch, a full review
should be performed.
In the following the two different reviews will be described, and in
the end, the different roles will be presented.

==OpenTissue Light Code Review==

The purpose of the light review is to make the code ready for
submission into the development branch.

It is not assumed that code in the development branch is meeting the
same strict requirements as the code in the release branch.
It is therefore not necessary to have as thorough a review as the
full review for code submitted to the development branch.

The light review is meant to be an easier review to perform, and very
much faster to do.

The aim for the light review is mostly to make sure that the look and feel of the code is up
to speed with OpenTissue, but it is assumed that at least some bugs or
problems are found during the light review.

In the light review there is only one person to do the review. This
person is designated [[Code_Review_Process#Reviewer|The Reviewer]], and the person who wrote the code
is designated [[Code_Review_Process#Author|The Author]].
The Author can ask either the OpenTissue-board or a person knowing OpenTissue to  be the reviewer, and the reviewer must
then review the code. For this purpose the reviewer uses the [[Media:Light_review_ver_1_1.pdf| light review report sheet]]

If the reviewer finds things that do not conform to OpenTissue in
accordance to the report sheet, or if the reviewer finds a bug or
problem in the code, then the reviewer compose a list of
errors/changes, and returns it to the author.
The author must now correct the code, and let the reviewer give it
another check. There can be a couple of iterations here before the
code is ready to be submitted to the development branch.

When the code is ready for submission to the development branch, the
reviewer completes the report sheet, and submits it to the OpenTissue
board.

==OpenTissue Full Code Review==

The Full Code Review is divided into two parts.
The first part is a documentation/code-cleanup review.
The second part is a bug fix/robustness/optimization review.
The reason for splitting the review into two parts, is to minimize
workload for the reviewers and at the same time separate things that
can be hard to do in a single review.
In the following, the two parts of the review is described.

===The code-cleanup review===

The first review that should be done is the code-cleanup review.
The purpose of this review is to make sure, that the look and feel of
the source lives up to the standards of OT. That is, stuff like
indention, variable names, code documentation, function documentation,
demo documentation, etc.
Also, it is a very large task to do both a documentation review and a
bug fix review at the same time.
Since OT is open source, and since it is a desire to have as many
programmers using OT as possible, it is essential to make it as easy for
users to understand the source code in order to start programming using OT,
quick and painless.

For this reason it is in everyones interest to have the code as well
documented as possible - and for this reason it is also a part of the
review to make sure that the documentation lives up to the standard of
OT.

The code standards that OT should conform to can be found [[Code_standards|here]].

The code-cleanup review is done in the following way:
When the programmer is finished with the code, and have done
everything in his power to make the source live up to the
OT-standards, the code is given to a reviewer, who is closely
familiar with the OT-standards. This reviewer will also act as
moderator in this part of the review.

The reviewer looks through the source in order to make sure that it
lives up to the standards. In the areas where it does not, it is
noted, and a list of things to be fixed is given back to the
programmer.

The programmer then fixes the items, and the reviewer examine the
fixes. If the reviewer finds that the source lives up to the standards,
the source is then given to a second reviewer.

The second reviewer then looks through the code and documentation, and
if he agrees that the source lives up to the standards, the source is
ready for the second review - the bug-fix review. If the second
reviewer thinks that the source does not live up to the OT-standards,
a list of fixes is given to the first reviewer.

If there is disagreement between the reviewers about whether the code
conforms to the standards, the two reviewers can call upon other reviewers.
Note, that the disagreement is hold internally between
OT-reviewers. This is done, because there is no reason to confuse the
programmer

===The bug-fix review===

After the code-cleanup review, the source should be easy to read and
understand.
This is necessary for this second review, which aims at finding bugs,
and areas of optimization. This can be looked at as a robustness
review - because the goal is to make the source as robust as possible.

The bug-fix review is instantiated by a moderator assigned by the
OT-board, and is done by two reviewers. Each reviewer have to read and
understand the code, and try to find areas of optimization, bugs
(eg. does the documentation say that the code should do one thing, and
the code does another), and areas where other parts of OT (or boost or
other dependency-lib) could be used with advantage.
After reading the code, the reviewers makes a list of things to
"fix". The reviewers should not them-self fix the code, but they may
suggest fixes.
These list can very well be comments in the source code.

The lists are given to the moderator (which can be one of the
reviewers), and the list of things to fix are discussed. This should be
done face to face, in order that no misunderstandings arise. When the
reviewers and moderator agrees which things to fix, the moderator
writes a list and give it to the programmer. This should also be done
face to face for the same reason as above.

When the programmer has fixed the source it is given back to the
reviewers for a second (quick) review to make sure that all fixes are
applied. If this is agreed upon, the source is ready for merge the release-branch.

The code should be frozen, or some other action should be taken to
make sure, that it is the correct revision of the code that is merged
into the release-branch.

It should be noted, that all code-reviews comments should be removed
from the code before a merge.

If a disagreement about the standard arises, the OT-board should also
be notified, because, clearly, the standards are ambiguous, and the
OT-board should take steps to fix the description in the standard.

==The work flow==

It is expected that an iteration of each part of the full review
process takes no longer than three weeks.
That is - from the author submits code to a review (either
code-cleanup or bug-fix) there must not proceed more than three
weeks before the author receives the result of the review.
It is the moderators responsibility that this deadline is kept.
Of course it may happen (e.g. during holidays) that this deadline can
not be kept.

==Roles==

There are a couple of different roles, which will be described in the
following:
* Author
* Moderator
* Reviewer
It should be noted, that during a light review, the moderator and the
reviewer will be the same person.

===Author===
The Author is the one who has written the code. It is the authors
responsibility to - in the first place - write good code which conform
to the OpenTissue standards.

When the author believe her code is in a final state, it is she who must submit
it to a review.

All changes to the code found during the review must be written into
the code by the author - that is - it is only the author who changes
the functionality of the code.

===Moderator===
The Moderator is the one who is responsible for the review-process.
It is the moderator who finds reviewers, and gives the code to the
reviewers. It is the responsibility of the moderator that deadlines
are met.

It is the moderator who is the one communicating with the author, and
as such it is him who compose defects/errors from the reviewer(s) into
a single list which are given back to the author.

When code are ready to be submitted to either the next review or into
a branch it is the moderators job to compose a report to the
OpenTissue board.

===Reviewer===
The Reviewer is the one who reads and comments on the code.
During the cleanup-process it is the responsibility of the reviewer to
make sure that the code becomes understandable.
During the bug-fix-process it is the responsibility of the reviewer to
make sure that as many bugs are found as possible, and that the code
conforms to the documentation.
The reviewer must submit his list of defects back to the moderator.
