$(document).ready(function () {
    setInterval("updateAll()", 100);
});

function randomString(length) {
    const chars = '0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ';
    let result = '';
    for (let i = length; i > 0; --i) result += chars[Math.floor(Math.random() * chars.length)];
    return result;
}

let filters = {};
let images = {};
let watches = {};

function updateAll() {
    updateLog();
    updateFilter();
    updateWatches();
}

function updateLog() {
    let logDiv = $("#logs");
    let keepDown = false;
    if (logDiv[0].scrollTop + logDiv[0].clientHeight >= logDiv[0].scrollHeight - 200.0) {
        keepDown = true;
    }
    fetch("/log").then(res => {
        if (!res.ok) {
            throw new Error(res.status + "");
        }
        return res.text();
    }).then(data => {
        if (data === "") return;
        if (this.prev) {
            data = this.prev + data;
        }
        let logs = data.split('\n');
        for (let log of logs.slice(0, -1)) {
            $("#logs").append("<p>" + log + "</p>");
        }
        this.prev = logs[logs.length - 1];
    });
    if (keepDown) {
        logDiv[0].scrollTop = logDiv[0].scrollHeight - logDiv[0].clientHeight;
    }
}

function updateWatches() {
    fetch("/watch").then(res => res.json()).then(data => {
        for (let k in data) {
            if (watches.hasOwnProperty(k)) {
                watches[k].html(data[k]);
            } else {
                let line = $("<tr><td>" + k + "</td></tr>");
                let val = $("<td>" + data[k] + "</td>");
                watches[k] = val;
                line.append(val);
                $("#watches").append(line);
            }
        }
    })
}

function updateStatus() {
    fetch("/status").then(res => {
        if (!res.ok) {
            throw new Error(res.status + "");
        }
        return res.text();
    }).then(data => {
        $("#current_status").html(data);
    }).catch(err => {
        $("#current_status").html(`<p>${err}<p>`);
    });
}

function updateFilter() {
    fetch("/filter", {
        method: "POST"
    }).then(res => res.json()).then(data => {
        //data: [[uint64_t, false], [uint64_t, true], ...]
        for (let v of data) {
            if (filters[v[0]] === undefined) {
                let checkbox = $("<label class=\"mdui-list-item mdui-ripple\">" +
                    "<div class=\"mdui-list-item-content mdui-text-truncate\">" + v[0] + "</div>" +
                    "<div class=\"mdui-checkbox\">" +
                    "<input type=\"checkbox\" id=\"filter_" + v[0] + "\"/>" +
                    "<i class=\"mdui-checkbox-icon\"></i>" +
                    "</div>" +
                    "</label>");
                $("#filter").append(checkbox);
                let inner = $("#filter_" + v[0]);
                inner.change(function () {
                    filters[v[0]] = this.checked;
                    if (this.checked) $(images[v[0]]).show();
                    else $(images[v[0]]).hide();
                    fetch("/filter", {
                        method: "POST",
                        body: JSON.stringify(filters)
                    })
                });
            }
            $("#filter_" + v[0])[0].checked = v[1];
            if (!images.hasOwnProperty(v[0])) {
                let img = $("<img class=\"mdui-img-fluid image\" id=\"current_image\" src=\"\" alt=\"\"/>")
                img[0].src = "/img/" + v[0];
                img.on("click", function(e) {
                    let x = e.pageX - this.offsetLeft;
                    let y = e.pageY - this.offsetTop;
                    fetch("/radar", {
                        method: "POST",
                        body: JSON.stringify([x, y])
                    })
                });
                images[v[0]] = img[0];
                $("#images").append(img);
            }

            filters[v[0]] = v[1];

        }
    });
}

function exitServer() {
    window.location.href = "/exit";
}
