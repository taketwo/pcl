#!/usr/bin/env python3

"""
Software License Agreement (BSD License)

 Point Cloud Library (PCL) - www.pointclouds.org
 Copyright (c) 2018-, Open Perception, Inc.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holder(s) nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
"""

import sys
import argparse
from collections import defaultdict
import re

import requests


def fetch(start, end):
    url = "https://api.github.com/search/issues?q=is:pr+repo:PointCloudLibrary/pcl"
    if end is None:
        url += "+merged:>=" + start
    else:
        url += "+merged:" + start + ".." + end
    pr_info = []
    page = 1
    total_count = 1  # for the first iteration assume that total is 1
    while len(pr_info) < total_count:
        response = requests.get(url + "&page=" + str(page))
        data = response.json()
        if response.status_code != 200:
            sys.exit(
                "Failed to fetch PR info. Server response: '{}'".format(data["message"])
            )
        total_count = data["total_count"]
        for item in data["items"]:
            print(item["number"], item["title"], [n["name"] for n in item["labels"]])
            pr_info.append(
                {"id": item["number"], "title": item["title"], "labels": item["labels"]}
            )
        page += 1
    return pr_info


def generate_text_content(tag, pr_info):

    module_order = (
        None,
        "cmake",
        "2d",
        "common",
        "cuda",
        "features",
        "filters",
        "geometry",
        "gpu",
        "io",
        "kdtree",
        "keypoints",
        "ml",
        "octree",
        "outofcore",
        "people",
        "recognition",
        "registration",
        "sample_consensus",
        "search",
        "segmentation",
        "simulation",
        "stereo",
        "surface",
        "visualization",
        "apps",
        "docs",
        "tutorials",
        "examples",
        "tests",
        "tools",
        "ci",
    )

    module_titles = {
        None: "Uncategorized",
        "2d": "libpcl_2d",
        "apps": "PCL Apps",
        "cmake": "CMake",
        "ci": "CI",
        "common": "libpcl_common",
        "cuda": "libpcl_cuda",
        "docs": "PCL Docs",
        "examples": "PCL Examples",
        "features": "libpcl_features",
        "filters": "libpcl_filters",
        "gpu": "libpcl_gpu",
        "io": "libpcl_io",
        "kdtree": "libpcl_kdtree",
        "keypoints": "libpcl_keypoints",
        "ml": "libpcl_ml",
        "octree": "libpcl_octree",
        "outofcore": "libpcl_outofcore",
        "people": "libpcl_people",
        "recognition": "libpcl_recognition",
        "registration": "libpcl_registration",
        "sample_consensus": "libpcl_sample_consensus",
        "search": "libpcl_search",
        "segmentation": "libpcl_segmentation",
        "simulation": "libpcl_simulation",
        "stereo": "libpcl_stereo",
        "surface": "libpcl_surface",
        "tests": "PCL Tests",
        "tools": "PCL Tools",
        "tutorials": "PCL Tutorials",
        "visualization": "libpcl_visualization",
    }

    changes_order = ("new-feature", "deprecation", "removal", "behavior", "api", "abi")

    changes_titles = {
        "new-feature": "New Features",
        "deprecation": "Deprecated",
        "removal": "Removed",
        "behavior": "Behavioral changes",
        "api": "API changes",
        "abi": "ABI changes",
    }

    changes_description = {
        "new-feature": "Newly added functionalities.",
        "deprecation": "Deprecated code scheduled to be removed after two minor releases.",
        "removal": "Removal of deprecated code.",
        "behavior": "Changes in the expected default behavior.",
        "api": "Changes to the API which didn't went through the proper deprecation and removal cycle.",
        "abi": "Changes that cause ABI incompatibility but are still API compatible.",
    }

    changes_labels = {
        "breaks API": "api",
        "breaks ABI": "abi",
        "behavior": "behavior",
        "deprecation": "deprecation",
        "removal": "removal",
    }

    # change_log content
    clog = []

    # Infer version from tag
    #  version = extract_version(tag)

    # Find the commit date for writting the Title
    cmd = ("git log -1 --format=%ai " + tag).split()
    #  output = subprocess.run(cmd, cwd=FOLDER, stdout=subprocess.PIPE)
    #  date = output.stdout.split()[0].decode()
    #  tokens = date.split("-")
    #  clog += [
    #  "## *= "
    #  + version
    #  + " ("
    #  + tokens[2]
    #  + "."
    #  + tokens[1]
    #  + "."
    #  + tokens[0]
    #  + ") =*"
    #  ]

    # Map each PR into the approriate module and changes
    modules = defaultdict(list)
    changes = defaultdict(list)
    module_re = re.compile(r"module: \S+")
    changes_re = re.compile(r"changes: ")
    feature_re = re.compile(r"new feature")

    for pr in pr_info:

        pr["modules"] = []
        pr["changes"] = []

        for label in pr["labels"]:
            if module_re.fullmatch(label["name"]):
                module = label["name"][8:]
                pr["modules"].append(module)
                modules[module].append(pr)

            elif changes_re.match(label["name"]):
                key = changes_labels[label["name"][9:]]
                pr["changes"].append(key)
                changes[key].append(pr)

            elif feature_re.fullmatch(label["name"]):
                pr["changes"].append("new-feature")
                changes["new-feature"].append(pr)

        # No labels defaults to section None
        if not pr["modules"]:
            modules[None].append(pr)
            continue

    # Generate Changes Summary
    for key in changes_order:

        # Skip empty sections
        if not changes[key]:
            continue

        clog += ["\n### `" + changes_titles[key] + ":`\n"]

        clog += ["*" + changes_description[key] + "*\n"]

        for pr in changes[key]:
            prefix = "".join(["[" + k + "]" for k in pr["modules"]])
            if prefix:
                prefix = "**" + prefix + "** "
            clog += [
                "* "
                + prefix
                + pr["title"]
                + " [[#"
                + str(pr["id"])
                + "]]"
                + "(https://github.com/PointCloudLibrary/pcl/pull/"
                + str(pr["id"])
                + ")"
            ]

    # Traverse Modules and generate each section's content
    clog += ["\n### `Modules:`"]
    for key in module_order:

        # Skip empty sections
        if not modules[key]:
            continue

        # if key:
        clog += ["\n#### `" + module_titles[key] + ":`\n"]

        for pr in modules[key]:
            prefix = "".join(["[" + k + "]" for k in pr["changes"]])
            if prefix:
                prefix = "**" + prefix + "** "
            clog += [
                "* "
                + prefix
                + pr["title"]
                + " [[#"
                + str(pr["id"])
                + "]]"
                + "(https://github.com/PointCloudLibrary/pcl/pull/"
                + str(pr["id"])
                + ")"
            ]

    return clog


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="""
    Generate changelog from pull requests merged during a given period.
    """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("period", type=str, nargs="*", help="time period")
    args = parser.parse_args()

    if len(args.period) > 2:
        sys.exit("Period should contain no more than two values (start/end)")
    if len(args.period) == 2:
        period_begin, period_end = args.period
    if len(args.period) == 1:
        period_begin = args.period[0]
        period_end = None
    if len(args.period) == 0:
        period_begin = None
        period_end = None

    pr_info = fetch(period_begin, period_end)

    clog = generate_text_content(period_end, pr_info=pr_info)
    print("\n".join(clog))
