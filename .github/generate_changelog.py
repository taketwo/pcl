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
import pickle

import requests


SKIP_LIST = [
    3415,
    3381,
    3358,
    3324,
    3201,
    2940,
    2935,
    2928,
    2693,
    2681,
    2659,
    3520,
    3083,
    3357,
    3103,
    3371,
    3167,
    2783,
    2631,
    2650,
    2695,
    3156,
    3037,
    2914,
    2815,
    3359,
    2820,
    2773,
    3110,
    3493,
    3172
]

GROUPS = [
    (
        "Prefer lambdas over binds",
        [3189, 3178, 3199, 3136, 3192, 3209, 3231, 3243, 3173, 3171, 3254],
        ["modernization"],
    ),
    (
        "Prefer range-based for loops",
        [
            2812,
            2834,
            2835,
            2836,
            2838,
            2839,
            2840,
            2841,
            2842,
            2843,
            2844,
            2845,
            2846,
            2847,
            2848,
            2849,
            2850,
            2851,
            2853,
            2854,
            2855,
            2856,
            2857,
            2858,
            2859,
            2860,
            3396,
            2837,
            2887,
        ],
        ["modernization"],
    ),
    (
        "Prefer `nullptr` over 0 and `NULL`",
        [
            3004,
            3005,
            3006,
            3007,
            3008,
            3009,
            3010,
            3011,
            3012,
            3013,
            3014,
            3015,
            3016,
            3017,
            3018,
            3019,
            3020,
            3021,
            3022,
            3023,
            3024,
            3025,
            3026,
            3027,
            3028,
            3029,
        ],
        ["modernization"],
    ),
    ("Migrate to `std::chrono`", [2913, 2919, 3318], ["modernization"]),
    (
        "Migrate from `boost::thread` to `std::thread`",
        [3060, 3094],
        ["modernization", "changes: breaks ABI"],
    ),
    (
        "Migrate `mutex`, `lock` and `csv` to modernization",
        [3078, 3074, 3068, 3063, 3086, 3084, 3093, 3091, 3088, 3100],
        ["modernization"],
    ),
    (
        "Prefer `using` over `typedef`",
        [
            3112,
            3113,
            3115,
            3117,
            3118,
            3121,
            3122,
            3123,
            3124,
            3125,
            3129,
            3130,
            3132,
            3134,
            3137,
            3138,
            3139,
            3144,
        ],
        ["modernization"],
    ),
    (
        "Prefer `std` math functions over C functions",
        [3287, 3282, 3280, 3270, 3258, 3257, 3256, 3255, 3271, 3087, 3236, 3272],
        ["modernization"],
    ),
    (
        "Prefer using `Ptr` typedefs and migrate to `std` smart pointers in non-API code",
        [3061, 3141, 3217, 3474, 3482, 3486, 3489, 3497, 2929, 2823, 2821, 2804],
        ["modernization", "changes: breaks ABI"],
    ),
    (
        "Migrate to `std` random number generators",
        [2956, 3069, 2962],
        ["modernization", "changes: breaks ABI"],
    ),
    (
        "Deprecate `pcl_isnan`, `pcl_isfinite`, and `pcl_isinf` in favor of `std` methods",
        [2798, 3457],
        ["modernization", "changes: deprecation"],
    ),
    (
        "Add explicit `std::` prefix to standard types/functions",
        [3328, 3327, 3326, 3265],
        ["modernization"],
    ),
    (
        "Remove `else` after `return` statement",
        [3186, 3185, 3184, 3182, 3180, 3181, 3183],
        ["modernization"],
    ),
    ("Remove redundant `typename` keyword", [2927, 2897, 2896], ["modernization"]),
    (
        "Prefer `#pragma once` over `#ifndef` include guards",
        [2707, 2617],
        ["modernization"],
    ),
    (
        "Apply clang-format to white-listed modules",
        [3416, 3393, 3363, 3356, 3344, 3343],
        ["modernization"],
    ),
    (
        "Modernize FLANN finder script",
        [3317, 3220, 3202, 3157, 2910, 2905, 2861, 2740],
        ["module: cmake"],
    ),
    ("Remove default constructors/destructors", [3440, 3454], ["modernization"]),
    (
        "Fix various compiler warnings",
        [2778, 2775, 2782, 2898, 2907, 3342, 3409, 3377, 3208, 3375, 3372, 3385, 2781, 3075, 3001, 2665],
        ["modernization"],
    ),
    (
        "Restructure and add functionality to filters templated on `PCLPointCloud2`",
        [3500, 3483],
        ["module: filters", "changes: breaks ABI", "changes: deprecation"],
    ),
]


def group_prs(pr_info):
    grouped = []
    for g in GROUPS:
        grouped += g[1]
    new_pr_info = [pr for pr in pr_info if pr["id"] not in grouped]
    for title, prs, label in GROUPS:
        labels = [{"name": l} for l in label]
        new_pr_info.append({"id": prs, "title": title, "labels": labels})
    return new_pr_info


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
        response = requests.get(url + "&page=" + str(page) + "&per_page=100")
        data = response.json()
        if response.status_code != 200:
            sys.exit(
                "Failed to fetch PR info. Server response: '{}'".format(data["message"])
            )
        total_count = data["total_count"]
        for item in data["items"]:
            #  print(item["number"], item["title"], [n["name"] for n in item["labels"]])
            pr_info.append(
                {"id": item["number"], "title": item["title"], "labels": item["labels"]}
            )
        page += 1
    return pr_info


def make_pr_bullet_point(pr, prefix):
    if isinstance(pr["id"], list):
        ids = pr["id"]
    else:
        ids = [pr["id"]]
    refs = ", ".join(
        [
            "[#{0}](https://github.com/PointCloudLibrary/pcl/pull/{0})".format(i)
            for i in sorted(ids)
        ]
    )
    return "* " + prefix + pr["title"] + " [" + refs + "]"


def extract_version(tag):
    """Finds the corresponding version from a provided tag.
    If the tag does not correspond to a suitable version tag, the original tag
    is returned
    """
    version_re = re.compile(r"pcl-\S+")
    res = version_re.fullmatch(tag)

    # Not a usual version tag
    if not res:
        return tag

    return tag[4:]


def generate_text_content(tag, pr_info):

    module_order = (
        "modernization",
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
        "modernization": "Migration to C++14 and code modernization",
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
        "geometry": "libpcl_geometry",
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

    changes_order = (
        "new-feature",
        "deprecation",
        "removal",
        "behavior",
        "api",
        "abi",
    )

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
    #  tag = "pcl-1.9.1"
    #  version = extract_version(tag)

    # Find the commit date for writting the Title
    #  cmd = ("git log -1 --format=%ai " + tag).split()
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
    cpp14_re = re.compile(r"modernization")

    for pr in pr_info:

        if pr["id"] in SKIP_LIST:
            continue

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

            elif cpp14_re.fullmatch(label["name"]):
                pr["modules"].append("modernization")
                modules["modernization"].append(pr)

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
            clog += [make_pr_bullet_point(pr, prefix)]

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
            clog += [make_pr_bullet_point(pr, prefix)]

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

    if 1:
        pr_info = fetch(period_begin, period_end)
        pickle.dump(pr_info, open("dump", "wb"))
    else:
        pr_info = pickle.load(open("dump", "rb"))

    pr_info = group_prs(pr_info)

    clog = generate_text_content(period_end, pr_info=pr_info)
    print("\n".join(clog))
