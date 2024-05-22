/**
 * Core module - Basic library framework. Starts everything.
 */

var Pyr = {};

(function(Pyr) {
    var _initFuncs = [];
    var _resizeFuncs = [];
    var _qtApp = false;

    // Load iframe
    function _loadIframe(hash, call) {
        var $elem = $(hash + " iframe")
        var src = hash.substring(1) + ".html";

        if ($elem.attr("src") == "") {
            // Loading...
            $elem.before('<h4 class="text-center" id="iframe-load">Loading section data...</h4>');

            console.log("Pyr.core: Loading iFrame", hash)

            $elem.on("load", function() {
                console.log("Pyr.core: Finish loading Iframe", hash);
                $elem[0].contentWindow.postMessage(src, "*")

                if (typeof call === "function") {
                    call();
                }

                $("#iframe-load").remove();
            });

            $elem.attr("src", src);
        } else {
            console.log("Pyr.core: Opening already loaded Iframe", hash);

            if (typeof call === "function") {
                call();
            }
        }
    }

    // Methods to check the url hash and redirect the report to the section/component provided
    function _goTop() {
        $('html, body').animate({ scrollTop: 0 }, 0);
        setTimeout(function() { $('html, body').animate({ scrollTop: 0 }, 0) }, 50); // weird delay when opening a section hash on new tab click on browser
    }
    function _checkHash() {
        var hash = window.location.hash;

        try {
            if (Pyr._isMultiFile()) {
                // Code for multifile rendered report
                if (Pyr._inFrame()) {
                    if (hash !== "") {
                        console.log("Pyr.core: Changing parent hash to", hash)
                        window.location.hash = "";
                        parent.postMessage({ action: "changeHash", args: hash }, "*");
                    }
                } else {
                    if (hash == "") {
                        $('.nav-tabs a:first').tab('show');

                        if ($(hash + " iframe").attr("src") === "") {
                            try {
                                _loadIframe("#" + $("div.tab-content div:first")[0].id, function() {
                                    _goTop();
                                });
                            } catch(e) {
                                alertMsg("Error", "Can't found a section to init the report.")
                            }
                        }
                    } else {
                        hash = hash.split(".");
                        $('.nav-tabs a[href="' + hash[0] + '"]').tab('show');

                        _loadIframe(hash[0], function() {
                            // Check if the hash contains another element
                            if (hash.length > 1) {
                                $(hash[0] + " iframe")[0].contentWindow.postMessage({ action:"getOffset", id: hash[1] }, "*");
                            } else {
                                _goTop();
                            }
                        });
                    }
                }
            } else {
                // This works for singlefile
                if (hash == "") {
                    $('.nav-tabs a:first').tab('show');
                } else {
                    hash = hash.split(".");
                    $('.nav-tabs a[href="' + hash[0] + '"]').tab('show');

                    // Scroll to a component?
                    if (hash.length > 1) {
                        $('html, body').animate({ scrollTop: $("#" + hash[1]).offset().top }, 300);
                    } else {
                        _goTop();
                    }
                }

                // Fixes...
                Pyr._plotResetView(); //BUG in plot resizing. PROTIP: check if windows has been resized before firing this up
                Pyr.tableResetView(); // Bug on firefox
            }
        } catch(e) {
            console.error(e);
            Pyr.alertMsg("Error", "Invalid report URL");
        }
    }

    // Checks for multifile render elements
    Pyr._isMultiFile = function() {
        return Pyr._inFrame() || $("iframe").length > 0;
    }

    Pyr._inFrame = function() {
        return $(".page-header").length === 0;
    }

    // Function to register an init method for a module
    Pyr.registerInit = function(initFunc) {
        _initFuncs.push(initFunc);
    };

    // Function to register an init method for a module
    Pyr.registerResize = function(resFunc) {
        _resizeFuncs.push(resFunc);
    };

    // Waits for an element to be visible, then runs the callback
    Pyr.waitForElement = function(id, callback) {
        if (!$("#" + id).is(":visible")) {
            setTimeout(function() { Pyr.waitForElement(id, callback) }, 1000);
        } else {
            callback();
        }
    };

    // Detect element on a active tab
    Pyr.getVisible = function(selector) {
        if ($(".tab-pane.active").length > 0) {
            return $(".tab-pane.active " + selector);
        } else {
            return $(selector);
        }
    };

    // Strips HTML from a string
    Pyr.stripHTML = function(str) {
        var tmp = document.createElement("DIV");
        tmp.innerHTML = str;
        return tmp.textContent || tmp.innerText || "";
    };

    // Custom sorter function for tables, strips HTML from cells
    Pyr.tableSorter = function(a, b) {
        a = Pyr.stripHTML(a);
        b = Pyr.stripHTML(b);

        if ($.isNumeric(a) && $.isNumeric(b)) {
            // Convert numerical values from string to float.
            a = parseFloat(a);
            b = parseFloat(b);
        }

        if (a > b) return 1;
        if (a < b) return -1;
        return 0;
    };

    // Format numbers to show them nicely if too big or too small
    Pyr.autoFormat = function(num) {
        // Sci nums
        var absVal = Math.abs(num);

        if ((absVal < 0.009 && absVal > 0) || absVal >= 1000000) {
            return num.toExponential(2);
        } else {
            if (num === parseInt(num, 10)) {
                return num;
            }

            return num.toFixed(3);
        }
    };

    // Same but for the legends
    Pyr.autoformatLegend = function(num) {
        // Sci nums
        var absVal = Math.abs(num);

        if ((absVal < 0.009 && absVal > 0) || absVal >= 1000000) {
            return num.toExponential(2);
        } else {
            if (num === parseInt(num, 10)) {
                return num;
            }

            if (absVal < 1) {
                return num.toFixed(3);
            } else {
                return Math.round(num);
            }
        }
    };

    // Show an alert/info/error message
    Pyr.alertMsg = function(title, msg, buttons) {
        if (Pyr._inFrame()) {
            parent.postMessage({action: "alertMsg", args: {title: title, msg: msg, buttons: buttons}}, "*");
        } else {
            if (typeof title == "undefined") {
                $(".modal").modal("hide");
            } else {
                $('.modal p').html(" ");
                $('.modal strong').html(title);
                $('.modal p').html(msg);
                $('.modal').modal("show");
            }
        }
    };

    // Shows the help screen
    Pyr.help = function() {
        Pyr.alertMsg("Help", "<h3>Tables</h3><p>You can sort the tables by clicking on a " +
                "column title. Using the 'Search' or 'Filter' inputs you can search the " +
                "entire table or just a column. " +
                "For numerical columns, you can use the &lt; and &gt; operators to " + 
                "filter values less than or greater than a given value " + 
                "(for example: &lt;1.4 displays rows with values less than 1.4). " +
                "With the buttons on the toolbar you can toggle the list " +
                "view, useful to inspect large datasets on tiny screens or mobile devices " +
                "(<i class='glyphicon glyphicon-list-alt icon-list-alt'></i>), toggle the visible" +
                "columns (<i class='glyphicon glyphicon-th icon-th'></i>) or export the table to a " +
                "file (<i class='glyphicon glyphicon-export icon-share'></i>). " +
                "On multilevel header tables you can right-click " +
                "a column group to toggle its child columns or left-click to toggle the default " +
                "visible columns.<hr /><h3>Plots 2D</h3><p>You can zoom the " +
                "plots with the mouse wheel and pan the plot by dragging it with the mouse. " +
                "This behaviour can be changed using the controls on the top-right of the plot. " +
                "Double clicking the plot reset the scaling to the default. Individual " +
                "datasets can be toggled by clicking its name on the legend in the right.</p>" +
                "<p>Double clicking on a dataset name toggle all the other datasets. " +
                "Using the controls on the top-right you can export the plot to a <strong>.png</strong> file and also " +
                "the hover mode that shows the data under the cursor can be changed.</p>");
    };
    // Shows just the plot help screen
    Pyr.plotHelp = function() {
        Pyr.alertMsg("Help", "<h3>Plots 2D</h3><p>You can zoom the " +
                "plots with the mouse wheel and pan the plot by dragging it with the mouse. " +
                "This behaviour can be changed using the controls on the top-right of the plot. " +
                "Double clicking the plot reset the scaling to the default. Individual " +
                "datasets can be toggled by clicking its name on the legend in the right.</p>" +
                "<p>Double clicking on a dataset name toggle all the other datasets. " +
                "Using the controls on the top-right you can export the plot to a <strong>.png</strong> file and also " +
                "the hover mode that shows the data under the cursor can be changed.</p>");
    };

    // Launch an external app. This is meant to be executed under node.js environment
    Pyr.launchCommand = function(command, lock) {
        Pyr.alertMsg("Working", "Launching external process...");

        if (!_qtApp) { //QT report viewer checks links in app side.
            try {
                var exec = require("child_process").exec;

                exec(command, function callback(error, stdout, stderr) {
                    if (error) {
                        Pyr.alertMsg("Error", "Error executing command: " + command);
                    } else {
                        console.log("Stdout of " + command)
                        console.log(stdout);
                        setTimeout(function() { Pyr.alertMsg(); }, 1000);
                    }
                });

            } catch(e) {
                Pyr.alertMsg("Error", "External apps links not supported in this environment.");
            }
        }
    };

    //Set QT report viewer app launchers. This method is called by the QT app and also when the
    //column of tables are toggled.
    Pyr.setQTAppLaunchers = function() {
        _qtApp = true;

        $("a.pyr-launch").each(function() {
            var link = $(this);
            if (typeof link.attr("onclick") != "undefined") {
                var command = link.attr("onclick").split("'")[1]
                $(this).removeAttr("onclick").attr("href", "pyrapp:" + command);
            }
        })

        console.log("QT links ready");
    };

    // Checks if we are in QT environment
    Pyr.isQT = function() {
        return _qtApp;
    };

    // Checks if we are in a PDF rendered report, using a nice trick
    Pyr.isPDF = function() {
        return $(".tab-pane").css("page-break-inside") == "avoid";
    };


    // Init pyreport library
    $(window).load(function() {
        console.info("Pyr.core: Init");

        // Loading screen gone...
        $("#loading").hide();
        $(".invisible").removeClass("invisible");

        // Set internal linking handler
        $("a").click(function(e) {
            var newHash = $(this).attr("href");

            if (!$(this).hasClass("pyr-launch") && newHash[0] == "#") {
                if (!Pyr._inFrame()) { // Single file handling
                    location.hash = $(this).attr("href");
                    _checkHash();
                } else { // Multi file handling
                    parent.postMessage({action: "changeHash", args: newHash}, "*");
                }

                e.stopPropagation();
                return false
            }
        });

        // Container search
        $(".panel-heading .pull-right input").on("keyup", function() {
            var elem = this;
            var search = elem.value.toLowerCase();
            var components = $(elem).parents(".pyr-container").find(".pyr-component");

            components.each(function(i, el) {
                var name = $(el).attr('name').toLowerCase();

                if (name.indexOf(search) >= 0) {
                    $(el).show();
                } else {
                    $(el).hide();
                }
            });
        });

        //Tooltips
        $('[data-toggle="tooltip"]').tooltip();

        // Init registered modules
        for (var i in _initFuncs) {
            _initFuncs[i]();
        }

        // Resize callbacks
        $(window).resize(function(ev) {
            console.log("Pyr.core: Resizing window & running callbacks.")

            for (var i = 0; i < _resizeFuncs.length; ++i) {
                _resizeFuncs[i](ev);
            }
        });

        // Comunication - Used for content scaling and internal links using iframes
        window.addEventListener("message", function(ev) {
            if (!Pyr._inFrame()) { // I'm parent, do the requested action
                console.log("Pyr.core: Message received in parent window", ev, this);

                switch(ev.data.action) {
                    case "alertMsg":
                        var args = ev.data.args;
                        Pyr.alertMsg(args.title, args.msg, args.buttons);
                        break;
                    case "resize":
                        $("#" + ev.data.src.replace(".html", "") + " iframe").height(ev.data.args);
                        break;
                    case "scroll":
                        var iframeTop = $('.tab-pane.active iframe').position().top;
                        $('html, body').animate({ scrollTop: ev.data.args + iframeTop }, 300);
                        break;
                    case "changeHash":
                        window.location.hash = ev.data.args;
                        _checkHash();
                        break;
                    default:
                        console.log("Pyr.core: Message invalid");
                        break;
                }
            } else { // I'm iframe, send my data
                console.log("Pyr.core: Message received in iframe", ev, this);

                switch(ev.data.action) {
                    case "getOffset":
                        console.log("Pyr.core: getOffset action in iFrame");
                        var offset = $("#" + ev.data.id).offset().top;
                        ev.source.postMessage({action: "scroll", args: offset}, "*");
                        break;
                    default:
                        console.log("Pyr.core: getResize action in iFrame");
                        ev.source.postMessage({action: "resize", args: $(window.document.body).height(), src: ev.data}, "*");
                        break;
                }
            }
        }, false);

        // Done
        console.info("Pyr.core: Done");
        _checkHash();
    });
})(Pyr);

/**
 * CMatrix module - Implements Color Matrices based on SVG.
 */

(function(Pyr){
    // Draw a Cmatrix
    function _CM(id, obj) {
        var canvas = document.getElementById(id);

        // Some defaults
        //obj.layout.dragmode = "pan";
        obj.layout.hovermode = "closest";
        obj.layout.xaxis.showgrid = false;
        obj.layout.xaxis.zeroline = false;
        obj.layout.xaxis.showline = false;
        obj.layout.yaxis.showgrid = false;
        obj.layout.yaxis.zeroline = false;
        obj.layout.yaxis.showline = false;

        Plotly.plot(canvas, [obj.data],
            // Layout
            obj.layout,
            // Config
            {displayModeBar: !Pyr.isPDF(),
             displaylogo: false,
             scrollZoom: true,
             modeBarButtonsToRemove: ['sendDataToCloud', 'select2d', 'lasso2d']}).then(function(plotObj) {
                 // Check hash - lazy loading issue
                 var plotId = plotObj.id.replace("_plot", "");

                 if (window.location.hash === "#" + plotId) {
                     window.location.hash = "";
                     setTimeout(function() {
                         console.log("Pyr.Plot2d: Relocating to hash", plotId);
                         window.location.hash = plotId;
                     }, 500);
                 }
         });
    }

    // Init
    Pyr.registerInit(function() {
        console.info("Pyr.cmatrix: Init");

        // Transform each CM json component into a nice color matrix
        $(".cmatrix").each(function(i, cmatrix) {
            Pyr.waitForElement(cmatrix.id, function() {
                var id = cmatrix.id + "_data";
                var selector = cmatrix.id + "_plot";

                try {
                    _CM(selector, window[id]);
                } catch(e) {
                    Pyr.alertMsg("Error", "CMatrix '" + cmatrix.id + "' crashed!!." + e + "\n" + e.stack);
                    console.error("CMatrix '" + cmatrix.id + "' crashed CON" + e + "STACK" + e.stack);
                }
            });
        });

        console.info("Pyr.cmatrix: Done");
    });
})(Pyr);

/**
 * Legacy module - Fixes compatibility issues and other things. They are mostly
 * fixes for IE. Shame on you, IE.
 */

(function(Pyr){
    // Init
    Pyr.registerInit(function() {
        console.info("Pyr.legacy: Init");

        // No String.trim function
        if (typeof String.prototype.trim !== 'function') {
            String.prototype.trim = function() {
                return this.replace(/^\s+|\s+$/g, '');
            };
        }

        // No Array.indexOf function
        if (typeof Array.prototype.indexOf !== 'function') {
            Array.prototype.indexOf = function(obj, start) {
                 for (var i = (start || 0), j = this.length; i < j; i++) {
                     if (this[i] === obj) { return i; }
                 }

                 return -1;
            }
        };

        console.info("Pyr.legacy: Done");
    });
})(Pyr);

/**
 * Plots module - Implements 2d plots using d3.js library
 */

(function(Pyr) {
    // Setup the dynamic plots, using the json info in the component HTML.
    function _setPlot(i, plot) {
        var canvas = document.getElementById(plot.id + "_plot");
        var obj = window[plot.id + "_data"];

        // Defaults for layout in js code < smaller size for the report code
        obj.layout.dragmode = "pan";
        obj.layout.hovermode = "closest";
        obj.layout.legend = {traceorder: "normal"};

        // PDF settings
        if (Pyr.isPDF()) {
            obj.layout.legend.orientation = "h";
            obj.layout.legend.y = -0.2;
        }

        // Render plot svg
        Plotly.plot(canvas, obj.data, obj.layout,
            // Config
            {displayModeBar: !Pyr.isPDF(),
             displaylogo: false,
             scrollZoom: true,
             modeBarButtonsToAdd:[{name:"help",icon: Plotly.Icons.question, click: function(ev) {
                 Pyr.plotHelp();
             }}],
             modeBarButtonsToRemove: ['sendDataToCloud', 'select2d', 'lasso2d']}).then(function(plotObj) {
                 // After render

                 // Areas
                 if (obj.areas.length > 0) {
                     switch (obj.areas[2]) {
                         case "top":
                             _fillAreaToY(plotObj, obj.areas[0], obj.bound.y.max, obj.bound.x);
                             break;
                         case "bottom":
                             _fillAreaToY(plotObj, obj.areas[0], obj.bound.y.min, obj.bound.x);
                             break;
                         case "between":
                             var index1 = _getIndex(plotObj, obj.areas[0]);
                             var index2 = _getIndex(plotObj, obj.areas[1]);

                             Plotly.addTraces(plotObj, {
                                 y: obj.data[index2].y,
                                 x: obj.data[index2].x,
                                 type:"scatter", mode:"lines", line:{width:1,color:"#ccc"}, name: obj.areas[0] + "__area", showlegend: false}, index1);
                             Plotly.addTraces(plotObj, {
                                 y: obj.data[index1].y,
                                 x: obj.data[index1].x,
                                 type:"scatter", mode:"lines", line:{width:1,color:"#ccc"}, name: obj.areas[1] + "__area", showlegend: false}, index1);
                             Plotly.restyle(plotObj, {fillcolor:"rgba(150,150,150,0.3)", fill: "tonexty"}, _getIndex(plotObj, obj.areas[0] + "__area"));
                             plotObj.on("plotly_afterplot", function(a, b, c) {
                                 var index1 = _getIndex(plotObj, obj.areas[0]);
                                 var index2 = _getIndex(plotObj, obj.areas[1]);

                                 if ((plotObj.data[index1].visible === true || plotObj.data[index1].visible === undefined) &&
                                         (plotObj.data[index2].visible === true || plotObj.data[index2].visible === undefined)) {
                                     if (plotObj.data[0].visible !== true || plotObj.data[1].visible !== true) {
                                         Plotly.restyle(plotObj, {visible: true}, [0,1]);
                                     }
                                 } else {
                                     if ((plotObj.data[0].visible === true || plotObj.data[0].visible === undefined) &&
                                             (plotObj.data[1].visible === true || plotObj.data[1].visible === undefined)) {
                                         Plotly.restyle(plotObj, {visible: "legendonly"}, [0,1]);
                                     }
                                 }
                             })
                             break;
                         case "outside":
                             _fillAreaToY(plotObj, obj.areas[0], obj.bound.y.max, obj.bound.x);
                             _fillAreaToY(plotObj, obj.areas[1], obj.bound.y.min, obj.bound.x);
                             break;
                     }
                 }

                 // Check hash - lazy loading issue
                 var plotId = plotObj.id.replace("_plot", "");

                 if (window.location.hash === "#" + plotId) {
                     window.location.hash = "";
                     setTimeout(function() {
                         console.log("Pyr.Plot2d: Relocating to hash", plotId);
                         window.location.hash = plotId;
                     }, 500);
                 }
             }).catch(function(error) {
                 // Crashed
                 Pyr.alertMsg("Error", "Plot '" + plot.id + "' crashed");
                 console.error("Plot '" + plot.id + "' crashed", error);
             });
    }

    function _fillAreaToY(plot, index, y, xrange) {
        index = _getIndex(plot, index);

        Plotly.addTraces(plot, {
            y: [y, y], x: [xrange.min, xrange.max],
            type:"scatter", mode:"lines",
            line: { width:0, color:"#ccc" },
            showlegend: false
        }, index);

        Plotly.restyle(plot, {fill: "tonexty", fillcolor:"rgba(150, 150, 150, 0.3)"}, index + 1);
    }

    function _getIndex(plot, dataName) {
        for (i = 0; i < plot.data.length; ++i) {
            if (plot.data[i].name == dataName) {
                return i;
            }
        }

        return -1;
    }

    // Reset method called when you resize the window in another tab. TODO: refactor this into something private?
    Pyr._plotResetView = function() {
        var id = window.location.hash;

        $(id + " .plot2d, " + id + " .cmatrix").each(function(i, plot) {
            var chart = window[plot.id + "_plot"];

            if (typeof chart.data !== "undefined") {
                Plotly.Plots.resize(chart)
            }
        });

        console.log("Pyr.Plots: Updating plots & cmatrices.")
    }




    // Init
    Pyr.registerInit(function() {
        console.info("Pyr.plots: Init");

        // We will wait until the elements are visible to render the plots
        $(".plot2d").each(function(i, plot) {
            Pyr.waitForElement(plot.id, function() {
                _setPlot(i, plot);
            });
        });

        // Plots need to be adjusted when window is resized on another tab
        Pyr.registerResize(function() {
            console.log("Pyr.plot: Resize")
            Pyr._plotResetView();
        });

        // Expose d3 library
        window.d3 = Plotly.d3;

        // PATCH: extend d3.format to allow unit string
        var oldFormat = Plotly.d3.format;

        Plotly.d3.format = function(str) {
            var data = str.split("###");

            return function(val) {
                return oldFormat(str)(val) + " " + data[1];
            }
        }

        console.info("Pyr.plots: Done");
    });
})(Pyr);

/**
 * Tables module - Setups and improve table support across browsers. Basically, 
 * fixes tables height and set default visibility options for groups of columns
 */

(function(Pyr) {

    // Function that corrects headers widths and table height on fixed height mode.
    function _tableResetView() {
        $("table:visible").bootstrapTable("resetView");
        $("table[data-height]:visible").each(function(i, el) {
            var thisHeight = $(this).height();
            var contHeight = $(this).parents("div.bootstrap-table").height();
        });
    };

    // Get the group number of a th group
    function _getTHGroup(th) {
        var re = /group-([0-9]+)/;
        var str = th.attr("class");
        var m;

        if ((m = re.exec(str)) !== null) {
            if (m.index === re.lastIndex) {
                re.lastIndex++;
            }
            // View your result using the m-variable.
            // eg m[0] etc.
        }

        return +m[1];
    };

    // Get the span of a column group(based in span-x class present in element)
    function _getTableGroupSpan(th) {
        return 1 * th["class"].split("span-")[1].split(" ")[0];
    };

    // Makes the element blink.
    function _blink(el) {
        var cl = "#eeeeee";
        el.css("background-color", cl);
        setTimeout(function() { el.css("background-color", ""); }, 100);
    };

    Pyr.tableResetView = function() {
        _tableResetView();
    }




    // Init
    Pyr.registerInit(function() {
        console.info("Pyr.table: Init");

        // Fixes & options
        $("table").each(function() {
            var columns = $(this).bootstrapTable("getOptions").columns;

            // Visibility, for multi level header groups
            if (columns !== undefined && columns.length >= 2) {
                var visibles = [];
                var groupIndex = columns.length - 1;

                for (i in columns[groupIndex]) {
                    visibles.push(columns[groupIndex][i].visible);
                }

                $(this).bootstrapTable("getOptions").visibles = visibles;
            }

            // Custom search function(include RelOP search)
            $(this).bootstrapTable("getOptions").customSearch = function(query) {
                // Tools
                var getFieldIndex = function (columns, field) {
                    var index = -1;

                    $.each(columns, function (i, column) {
                        if (column.field === field) {
                            index = i;
                            return false;
                        }
                        return true;
                    });
                    return index;
                };

                var calculateObjectValue = function (self, name, args, defaultValue) {
                    var func = name;

                    if (typeof name === 'string') {
                        // support obj.func1.func2
                        var names = name.split('.');

                        if (names.length > 1) {
                            func = window;
                            $.each(names, function (i, f) {
                                func = func[f];
                            });
                        } else {
                            func = window[name];
                        }
                    }
                    if (typeof func === 'object') {
                        return func;
                    }
                    if (typeof func === 'function') {
                        return func.apply(self, args);
                    }
                    if (!func && typeof name === 'string' && sprintf.apply(this, [name].concat(args))) {
                        return sprintf.apply(this, [name].concat(args));
                    }
                    return defaultValue;
                };

                // Search
                var that = this;
                var s = this.searchText && this.searchText.toLowerCase();
                var f = $.isEmptyObject(this.filterColumns) ? null : this.filterColumns;
                var isRelationalSearch = false;

                if ("searchText" in that) {
                    isRelationalSearch = (that.searchText.charAt(0) == "<" || that.searchText.charAt(0) == ">");
                }

                // Check filter
                this.data = f ? $.grep(this.options.data, function (item, i) {
                    for (var key in f) {
                        if ($.isArray(f[key])) {
                            if ($.inArray(item[key], f[key]) === -1) {
                                return false;
                            }
                        } else if (item[key] !== f[key]) {
                            return false;
                        }
                    }
                    return true;
                }) : this.options.data;

                this.data = s ? $.grep(this.data, function (item, i) {
                    for (var key in item) {
                        key = $.isNumeric(key) ? parseInt(key, 10) : key;
                        var value = item[key],
                            column = that.columns[getFieldIndex(that.columns, key)],
                            j = $.inArray(key, that.header.fields);

                        // Fix #142: search use formatted data
                        if (column && column.searchFormatter) {
                            value = calculateObjectValue(column,
                                that.header.formatters[j], [value, item, i], value);
                            value = Pyr.stripHTML(value) // Strips HTML from searched field
                        }

                        var index = $.inArray(key, that.header.fields);
                        if (index !== -1 && that.header.searchables[index] && (typeof value === 'string' || typeof value === 'number')) {
                            if (isRelationalSearch) { // Relational OP search
                                var numVal = parseFloat(value);
                                var searchOP = s.charAt(0);
                                var searchVal = s.substring(1);
                                var searchVal = parseFloat(searchVal);

                                if (searchOP == "<") {
                                    if (numVal < searchVal) return true
                                } else if (searchOP == ">") {
                                    if (numVal > searchVal) return true
                                }
                            } else {
                                if (that.options.strictSearch) {
                                    if ((value + '').toLowerCase() === s) {
                                        return true;
                                    }
                                } else {
                                    if ((value + '').toLowerCase().indexOf(s) !== -1) {
                                        return true;
                                    }
                                }
                            }
                        }
                    }
                    return false;
                }) : this.data;
            }

            // Height
            $(this).on("post-header.bs.table", function() {
                var div = $(this).parent().prev();
                var height = div.children().outerHeight();
                div.height(height);
            });

            // Events for after updating a Table
            $(this).on("all.bs.table", function() {
                // QT links
                if (Pyr.isQT()) {
                    Pyr.setQTAppLaunchers();
                }

                // Tooltips
                $(".pyr-comment:visible").tooltip();
            });
        });

        // Right click event for toggle group columns visibility by column
        $(".bootstrap-table").on("contextmenu", function(ev) {
            var elem = $(ev.target).first();

            if (!elem.hasClass("th-inner")) {
                elem = elem.find(".th-inner");
            }

            if (elem.hasClass("th-inner")) {
                $("body").off("click");
                $("#table-group-menu").remove();

                var th = elem.parent();

                if (th.hasClass("group")) {
                    var table = th.parents(".bootstrap-table").find("table[data-toggle]");
                    var clickedRow = th.parent("tr").index();
                    var columns = table.bootstrapTable("getOptions").columns[clickedRow];
                    var clickedIndex = _getTHGroup(th);
                    var thisSpan = _getTableGroupSpan(columns[clickedIndex]);
                    var prevSpan = 0;

                    for (i = 1; i < clickedIndex; ++i) {
                        prevSpan += _getTableGroupSpan(columns[i]);
                    }

                    var container = elem.parents("div.bootstrap-table");
                    var menu = container.find(".dropdown-menu").first().clone();
                    menu.attr("id", "table-group-menu");
                    var parentOffset = $(this).offset();
                    var relX = ev.pageX - parentOffset.left;
                    var relY = ev.pageY - parentOffset.top;
                    menu.css({"top": relY, "left": relX});
                    menu.find("label").css({
                        display: "block",
                        padding: "3px 20px",
                        clear: "both",
                        "font-weight": 400,
                        "line-height": 1.428571429
                    });
                    menu.find("li input").on("click", function(ev) {
                        var UL = $(this).parents("div.bootstrap-table")
                                        .find(".dropdown-menu").first().find("li input");
                        var liIndex = $(this).parents("li").index();
                        $(UL[liIndex]).click();
                    });

                    menu.find("li").each(function(i, el) {
                        if (i < prevSpan || i > prevSpan * 1 + (thisSpan * 1) - 1) {
                            $(el).hide();
                        }
                    });

                    $("body").on("click", function(ev) {
                        if ($(ev.target).parents("ul").length == 0) {
                            $("body").off("click");
                            $("#table-group-menu").remove();
                        }
                    });

                    if (!container.hasClass("dropdown")) container.addClass("dropdown");
                    container.append(menu)
                    menu.show();

                    return false;
                }
            }
        });

        // Resize tables that are to wide on PDF renderer
        $(".bootstrap-table").each(function(a,b) {
            if (Pyr.isPDF()) {
                var diff = $(this).find(".sortable").length;

                if (diff > 10) {
                    $(this).addClass("pdf-table-zoom");
                } else if (diff > 15) {
                    $(this).addClass("pdf-table-max-zoom");
                }
            }
        });

        // Group toggle view
        $(".bootstrap-table").on("click", function(ev) {
            var elem = $(ev.target).first();

            if (!elem.hasClass("th-inner")) {
                elem = elem.find(".th-inner");
            }

            if (elem.hasClass("th-inner") && !elem.hasClass("sortable")) { // just groups columns
                // Check if group is all shown or not
                var firstGroupCol = 0;
                var th = elem.parent();
                var table = $(this).find(".table");
                var tableID = table.parents(".table").attr("id") +  "_";
                var visibles = table.bootstrapTable("getOptions").visibles;

                // Get initial group column index and number of group columns
                var clickedRow = th.parent("tr").index();
                var columns = table.bootstrapTable("getOptions").columns[clickedRow];
                var clickedIndex = _getTHGroup(th);

                for (i = 1; i < clickedIndex; ++i) {
                    firstGroupCol += _getTableGroupSpan(columns[i]);
                }

                var groupTotalCols = _getTableGroupSpan(columns[clickedIndex]);
                var allVisible = true;
                var hidden = table.bootstrapTable("getHiddenColumns");

                // Check if all group columns are visible
                for (i in hidden) {
                    var index = hidden[i].fieldIndex - 1
                    if (index >= firstGroupCol && index < firstGroupCol + groupTotalCols) {
                        allVisible = false;
                    }
                }

                if (allVisible) {
                    for (i = firstGroupCol; i < firstGroupCol + groupTotalCols; ++i) {
                        if (visibles[i + 1]) {
                            table.bootstrapTable("showColumn", tableID + "col_" + i)
                        } else {
                            table.bootstrapTable("hideColumn", tableID + "col_" + i);
                        }
                    }
                } else {
                    for (i = firstGroupCol; i < firstGroupCol + groupTotalCols; ++i) {
                        table.bootstrapTable("showColumn", tableID + "col_" + i);
                    }
                }

                // Visual cue
                var blinkIndex = clickedIndex;

                if (columns[blinkIndex].visible) {

                    for (i = 0; i < clickedIndex; ++i) {
                        if (!columns[i].visible) blinkIndex--;
                    }

                    var blinkEl = table.find("tr").eq(clickedRow).find("th").eq(blinkIndex);
                    _blink(blinkEl);
                }
            }
        });

        // Set the window resize event to fix widths of headers
        Pyr.registerResize(function() {
            console.log("Pyr.table: Resize")
            _tableResetView();
        });

        // Run one time, improves PDF rendering
        _tableResetView();

        console.info("Pyr.table: Done");
    });
})(Pyr);
