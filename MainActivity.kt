package com.wan.bilik

import android.annotation.SuppressLint
import android.graphics.Color
import android.os.Bundle
import android.webkit.JavascriptInterface
import android.webkit.WebView
import android.webkit.WebViewClient
import androidx.appcompat.app.AppCompatActivity
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.net.HttpURLConnection
import java.net.URL
import android.content.res.Configuration

class MainActivity : AppCompatActivity() {

    private lateinit var webView: WebView

    private val primaryUrl = "http://192.168.1.4/state"
    private var currentApiUrl = ""

    @SuppressLint("SetJavaScriptEnabled")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        webView = findViewById(R.id.webview)
        webView.settings.javaScriptEnabled = true

        webView.webViewClient = object : WebViewClient() {
            override fun onPageFinished(view: WebView?, url: String?) {
                super.onPageFinished(view, url)
                val isNightMode = resources.configuration.uiMode and Configuration.UI_MODE_NIGHT_MASK == Configuration.UI_MODE_NIGHT_YES
                webView.evaluateJavascript("setDarkMode($isNightMode)", null)
            }
        }

        webView.addJavascriptInterface(WebAppInterface(), "Android")

        CoroutineScope(Dispatchers.IO).launch {
            currentApiUrl = try {
                val url = URL(primaryUrl)
                val connection = url.openConnection() as HttpURLConnection
                connection.connectTimeout = 3000
                connection.readTimeout = 3000
                connection.requestMethod = "HEAD"
                connection.connect()
                val result = if (connection.responseCode == HttpURLConnection.HTTP_OK) {
                    val base = url.protocol + "://" + url.host + (if (url.port != -1) ":${url.port}" else "") + "/"
                    base
                } else {
                    ""
                }
                connection.disconnect()
                result
            } catch (_: Exception) {
                ""
            }

            withContext(Dispatchers.Main) {
                webView.loadUrl("file:///android_asset/index.html")
            }
        }
    }

    override fun onStart() {
        super.onStart()
        webView.evaluateJavascript("triggerRefresh();", null)
    }

    inner class WebAppInterface {
        @JavascriptInterface
        fun getApiBaseUrl(): String {
            return currentApiUrl
        }

        @JavascriptInterface
        fun setStatusBarColor(colorString: String) {
            runOnUiThread {
                try {
                    val color = Color.parseColor(colorString)
                    window.statusBarColor = color
                } catch (_: IllegalArgumentException) {
                }
            }
        }
    }
}
